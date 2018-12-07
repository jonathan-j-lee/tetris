"""
env -- Module managing the state of the environment.
"""

from __future__ import division, generators, print_function, unicode_literals
import numpy as np
import rospy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from solver import TILE_TYPES
from tf.transformations import quaternion_from_euler, quaternion_multiply


class Environment:
    def __init__(queue_size=5):
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

    def marker_frame(self, marker_id):
        return 'ar_track_{}'.format(marker_id)


class PNPEnvironment:
    FRAME_MARKER_ID = 7

    def __init__(self, queue_size=5, table_height=np.nan, frame_id='base',
                 tool_frame_id='right_gripper'):
        super(Environment, self).__init__(queue_size=queue_size)
        self.table_height = table_height
        self.frame_id, self.tool_frame_id = frame_id, tool_frame_id

    def get_rel_transform(self, frame):
        return self.get_transform(frame, self.frame_id)

    def get_gripper_transform(self):
        return self.get_rel_transform(self.tool_frame_id)

    def find_tile_center(self, tile_name):
        tile_type = TILE_TYPES[tile_name]
        marker = self.marker_frame(tile_type.marker_id)
        trans = self.lookup_transform(self.frame_id, marker)
        if trans is not None:
            offset = PoseStamped()
            offset.header.frame_id = marker
            pos, orien = offset.pose.position, offset.pose.orientation
            pos.x, pos.y, pos.z = tile_type.x_offset, tile_type.y_offset, 0
            orien.x, orien.y, orien.z, orien.w = 0, 0, 0, 1

            rot = trans.translation.rotation
            _, _, e_z = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            rot.x, rot.y, rot.z, rot.w = quaternion_from_euler([0, 0, e_z])

            center = do_transform_pose(offset, trans)
            pos, orien = center.position, center.orientation
            return (np.array([pos.x, pos.y, pos.z]),
                    np.array([orien.x, orien.y, orien.z, orien.w]))

    # def find_table(self):
    #     self.get_transform()
