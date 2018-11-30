#!/usr/bin/env python

from __future__ import division, generators, print_function, unicode_literals

import actionlib
import rospy
from moveit_msgs.msg import MoveGroupAction

def main():
    rospy.init_node('grasp')
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    client.wait_for_server()

if __name__ == '__main__':
    main()
