"""
positioning --
"""

from __future__ import division, generators, print_function, unicode_literals

import rospy
from tf2_ros import Buffer, TransformListener, TransformException


class ARTracker:
    def __init__(self, queue_size=20):
        self.buffer = Buffer(queue)
        self.listener = TransformListener(self.buffer)

    def get_transform(self, target_frame, source_frame, timeout=5):
        start = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start > timeout:
            try:
                return self.buffer.lookup_transform(target_frame, source_frame, rospy.Time())
            except TransformException:
                pass

    def find_table(self):
        pass
