"""
env -- Module managing the state of the environment.
"""

from __future__ import division, generators, print_function, unicode_literals
import numpy as np
import rospy
from tf2_ros import Buffer, TransformListener, TransformException


class Environment:
    def __init__(queue_size=5):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, queue_size=queue_size)

    def get_transform(self, target_frame, source_frame, timeout=5):
        start = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start <= timeout:
            try:
                return self.buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            except TransformException:
                pass
        if rospy.get_param('verbose'):
            rospy.logwarn('Failed to find transform: "{}" -> "{}"'.format(
                source_frame, target_frame))


class PNPEnvironment:
    def __init__(self, queue_size=5, table_height=np.nan):
        super(Environment, self).__init__(queue_size=queue_size)
        self.table_height = table_height

    def find_table(self):
        pass
