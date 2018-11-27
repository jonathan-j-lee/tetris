#!/usr/bin/env python
"""
Pick and Place scriptt for Tetris Final Project
Author: James Fang
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint

from path_planner import *
from pick_and_place import *
from SuctionGripper import *
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper

def main():
    pap = PickAndPlace()  
    pap.add_obstacle("table", 0.765, -0.239, -0.316, 1.20, 1.10, 0.1)

    while not rospy.is_shutdown():
        x, y, z = 0.765, -0.248, -0.304 #0.816, -0.403, -0.170
        raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        z = pap.move_to_position_and_grasp(x, y, z)

        print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
        raw_input("Press <Enter> to move the right arm to goal pose 2: ")   
        pap.move_to_position(x=0.6, y=-0.3, z=0.0)

        raw_input("Press <Enter> to move the right arm to goal pose 3: ")
        pap.move_to_position_and_open(x, y, z)

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()

    """
    ## Add the obstacle to the planning scene here
    #LENGTHS
    X = 1.20
    Y = 1.10
    Z = 0.1
    box_size = np.array([X, Y, Z])
    box_name = "table"

    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    #x, y, and z position 0.787, -0.475, -0.172
    box_pose.pose.position.x = 0.787
    box_pose.pose.position.y = -0.475
    box_pose.pose.position.z = -0.174 - Z
    #Orientation as a quaternion
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0

    #planner.add_box_obstacle(box_size, box_name, box_pose)
    #planner.remove_obstacle(box_name)
    
    # #Create a path constraint for the arm (e.g. DOWN ORIENTATION CONSTRAINT here)
    orien_const = OrientationConstraint()
    orien_const.link_name = "left_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0; #down orientation
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    orientation_constraints = list() #[orien_const]
    """