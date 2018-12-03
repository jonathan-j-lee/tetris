#!/usr/bin/env python

from __future__ import division, generators, print_function, unicode_literals

import actionlib
import rospy
import tf2_ros
from moveit_msgs.msg import MoveGroupAction

PIECE_CODES = {
    't': 5,
    'bar': 4,
}

class Robot:
    GRIPPER_FRAME = 'left_hand'

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def locate(self, piece):
        codes = [value for name, value in PIECE_CODES.items() if piece == name]
        source_frame = 'ar_marker_{}'.format(codes[0])  # FIXME
        rospy.loginfo('Locating {}'.format(source_frame))
        return self.tf_buffer.lookup_transform(
            self.GRIPPER_FRAME, source_frame, rospy.Time())

def main():
    rospy.init_node('grasp', anonymous=True)

    # client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    # client.wait_for_server()

    robot = Robot()
    while not rospy.is_shutdown():
        try:
            print(robot.locate('bar'))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
            rospy.logerr(str(exc))
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
