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
    position = position if position is not None else DEFAULT_POSITION
    orientation = orientation if orientation is not None else DEFAULT_ORIENTATION
    target = PoseStamped()
    target.header.frame_id = frame_id
    target_pos, target_orien = target.pose.position, target.pose.orientation
    target_pos.x, target_pos.y, target_pos.z = position
    target_orien.x, target_orien.y, target_orien.z, target_orien.w = orientation
    return target


def convert_pose(pose):
    position, orientation = pose.pose.position, pose.pose.orientation
    return (np.array([position.x, position.y, position.z]),
            np.array([orientation.x, orientation.y, orientation.z, orientation.w]))


def log_pose(msg, position, orientation=None):
    """ Logs the given pose. """
    if orientation is None:
        rospy.loginfo('{} (x={}, y={}, z={})'.format(msg, *position))
    else:
        template = '{} (x={}, y={}, z={}, o_x={}, o_y={}, o_z={}, o_w={})'
        rospy.loginfo(template.format(msg, *np.concatenate((position, orientation))))


def marker_frame(marker_id):
    return 'ar_marker_{}'.format(marker_id)


def add_transform_offset(trans, translation):
    offset = create_pose(trans.child_frame_id, translation)
    return do_transform_pose(offset, trans)


class Environment:
    def __init__(self):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)

    def get_transform(self, source_frame, target_frame, timeout=5):
        frames = '"{}" w.r.t. "{}"'.format(source_frame, target_frame)
        if rospy.get_param('verbose'):
            rospy.loginfo('Acquiring transform: ' + frames)
        start = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start < timeout:
            try:
                return self.buffer.lookup_transform(target_frame, source_frame,
                                                    rospy.Time())
            except TransformException:
                pass
        if rospy.get_param('verbose'):
            rospy.logwarn('Failed to find transform: ' + frames)


class PNPEnvironment(Environment):
    FRAME_MARKER_ID = 7
    ROTATIONS = np.array([
        [0, 1, 0, 0],
        [1/2**0.5, 1/2**0.5, 0, 0],
        [1, 0, 0, 0],
        [1/2**0.5, -1/2**0.5, 0, 0],
    ])
    JOINT_NAMES = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
    ARM_JOINTS_NEUTRAL = {
        'left': np.array([-0.3762, 2.1852, 1.05500, -1.2966, 0.1848, 0.8326, 3.0158]),
        'right': np.array([0.2462, 2.2837, -0.8932, -1.3395, -3.0461, -0.5963, 2.9602]),
    }
    CAMERA_POSITIONS = np.array([
        [0.577, -0.108, 0.080],  # Board bottom left
        [0.638, 0.413, 0.183],  # Board top left
        [0.423, 0.600, 0.117],  # Board top right
        [0.250, 0.496, 0.204],  # Board bottom right
    ])

    def __init__(self, table_height=np.nan, frame_id='base',
                 tool_frame_id='right_gripper'):
        Environment.__init__(self)
        self.table_height = table_height
        self.frame_id, self.tool_frame_id = frame_id, tool_frame_id

    def get_rel_transform(self, frame):
        return self.get_transform(frame, self.frame_id)

    def get_gripper_transform(self):
        return self.get_rel_transform(self.tool_frame_id)

    def find_tile_center(self, tile_name):
        tile_type = TILE_TYPES[tile_name]
        trans = self.get_rel_transform(marker_frame(tile_type.marker_id))
        if trans is None:
            raise ValueError('Unable to obtain transform.')
        rot = trans.transform.rotation
        _, _, e_z = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        rospy.loginfo(str(trans.transform.translation))
        rot.x, rot.y, rot.z, rot.w = quaternion_from_euler(0, 0, e_z)
        offset = add_transform_offset(trans, [tile_type.x_offset, tile_type.y_offset, 0])
        return convert_pose(offset)

    def find_table(self):
        corners = ['board_top_left_marker', 'board_top_right_marker',
                   'board_bottom_right_marker', 'board_bottom_left_marker']
        offsets = [(0, 0), (-54, 0), (-54, 42), (0, 42)]
        for corner, (x_offset, y_offset) in zip(corners, offsets):
            marker = marker_frame(rospy.get_param(corner))
            trans = self.get_rel_transform(marker)
            if trans:
                return trans, x_offset, y_offset
        raise ValueError('Unable to find any board markers.')

    def find_slot_transform(self, tile):
        tile_size = rospy.get_param('tile_size')
        corner, x_offset, y_offset = self.find_table()
        tile_type = TILE_TYPES[tile.tile_name]
        pattern = rotate(tile_type.pattern, tile.rotations)
        for row in range(pattern.shape[0]):
            for column in range(pattern.shape[1]):
                if pattern[row, column] == AR_TAG:
                    break
        else:
            raise ValueError('AR tag not found in pattern.')

        row, column = row + tile.row, column + tile.column
        x_offset += column*tile_size
        y_offset -= row*tile_size
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
        offset = add_transform_offset(corner, np.array([x_offset, y_offset, 0]))
        return convert_pose(offset)
