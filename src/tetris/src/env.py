"""
env -- Module managing the state of the environment.
"""

from __future__ import division, generators, print_function
import numpy as np
import rospy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from solver import TILE_TYPES, AR_TAG
from tf.transformations import quaternion_from_euler, euler_from_quaternion


DEFAULT_POSITION = np.zeros(3)
DEFAULT_ORIENTATION = np.array([0, 0, 0, 1])


def create_pose(frame_id, position=None, orientation=None):
    """ Convert a pose as arrays into a ROS-compatible timestamped pose data type. """
    position = position or DEFAULT_POSITION
    orientation = orientation or DEFAULT_ORIENTATION
    target = PoseStamped()
    target.header.frame_id = frame_id
    target_pos, target_orien = target.pose.position, target.pose.orientation
    target_pos.x, target_pos.y, target_pos.z = position
    target_orien.x, target_orien.y, target_orien.z, target_orien.w = orientation
    return target


def convert_pose(pose):
    position, orientation = pose.position, pose.orientation
    return (np.array([position.x, position.y, position.z]),
            np.array([orientation.x, orientation.y, orientation.z, orientation.w]))


def log_pose(msg, position, orientation=None):
    """ Logs the given pose. """
    if not orientation:
        rospy.loginfo('{} (x={}, y={}, z={})'.format(msg, *position))
    else:
        template = '{} (x={}, y={}, z={}, o_x={}, o_y={}, o_z={}, o_w={})'
        rospy.loginfo(template.format(msg, *np.concatenate((position, orientation))))


def marker_frame(marker_id):
    return 'ar_track_{}'.format(marker_id)


def add_transform_offset(trans, translation):
    offset = create_pose(trans.child_frame_id, translation)
    return do_transform_pose(offset, trans)


class Environment:
    def __init__(self, queue_size=5):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, queue_size=queue_size)

    def get_transform(self, target_frame, source_frame, timeout=5):
        try:
            return self.buffer.lookup_transform(target_frame, source_frame,
                                                rospy.Time(), rospy.Duration(timeout))
        except TransformException:
            if rospy.get_param('verbose'):
                template = 'Failed to find transform: "{}" -> "{}"'
                rospy.logwarn(template.format(source_frame, target_frame))


class PNPEnvironment(Environment):
    FRAME_MARKER_ID = 7
    ROTATIONS = np.array([
        [0, 1, 0, 0],
        [1/2**0.5, 1/2**0.5, 0, 0],
        [1, 0, 0, 0],
        [1/2**0.5, -1/2**0.5, 0, 0],
    ])

    def __init__(self, queue_size=5, table_height=np.nan, frame_id='base',
                 tool_frame_id='right_gripper'):
        Environment.__init__(self, queue_size=queue_size)
        self.table_height = table_height
        self.frame_id, self.tool_frame_id = frame_id, tool_frame_id

    def get_rel_transform(self, frame):
        return self.get_transform(frame, self.frame_id)

    def get_gripper_transform(self):
        return self.get_rel_transform(self.tool_frame_id)

    def find_tile_center(self, tile_name):
        tile_type = TILE_TYPES[tile_name]
        trans = self.get_rel_transform(marker_frame(tile_type.marker_id))
        if not trans:
            raise ValueError('Unable to obtain transform.')
        rot = trans.translation.rotation
        _, _, e_z = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        rot.x, rot.y, rot.z, rot.w = quaternion_from_euler([0, 0, e_z])
        offset = add_transform_offset(trans, [tile_type.x_offset, tile_type.y_offset, 0])
        return convert_pose(offset)

    def find_table(self):
        # TODO
        return self.get_rel_transform(marker_frame(rospy.get_param('board_top_left_marker')))

    def find_slot_transform(self, tile):
        tile_size = rospy.get_param('tile_size')
        top_left, tile_type = self.find_table(), TILE_TYPES[tile.tile_name]
        pattern = rotate(tile_type.pattern, tile.rotations)
        for row in range(pattern.shape[0]):
            for column in range(pattern.shape[1]):
                if pattern[row, column] == AR_TAG:
                    break
        else:
            raise ValueError('AR tag not found in pattern.')

        row, column = row + tile.row, column + tile.column
        x_offset = column*tile_size
        y_offset = row*tile_size
        # TODO: refactor
        if rotations == 0:
            x_offset += tile_type.x_offset
            y_offset += tile_type.y_offset
        elif rotations == 1:
            x_offset += tile_type.y_offset
            y_offset -= tile_type.x_offset
        elif rotations == 2:
            x_offset -= tile_type.x_offset
            y_offset -= tile_type.y_offset
        else:
            x_offset -= tile_type.y_offset
            y_offset += tile_type.x_offset
        offset = add_transform_offset(top_left, np.array([x_offset, y_offset, 0]))
        return convert_pose(offset)
