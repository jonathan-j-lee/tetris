#!/usr/bin/env python
"""
Pick and Place scriptt for Tetris Final Project
Author: James Fang
"""

import sys
import rospy
import numpy as np
import tf2_ros 

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import TransformStamped

from path_planner import *
from pick_and_place import *
from SuctionGripper import *
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper

def main():
    """
        z :
            0       : 0
            pi / 2  : sqrt(2) / 2
            pi      : 1
            3pi / 2 : sqrt(2) / 2
        w :
            0       : 1
            pi / 2  : sqrt(2) / 2
            pi      : 0
            3pi / 2 : -sqrt(2) / 2

        0       : 0,    1.0,    0,  0
        90      : r2,   r2,     0,  0
        180     : 1.0,  0,      0,  0
        270     : r2,   -r2,    0,  0
    """
    x, y, z = 0.864, -0.244, 0.027
    r2 = np.sqrt(2) / 2
    rotations = [
        (0,     1.0,     0,      0),
        (r2,    r2,     0,      0),
        (1.0,   0.0,     0,      0),
        (r2,    -r2,    0,      0)
    ]
    o_x, o_y, o_z, o_w = rotations[0]
    endx, endy = x, y

    #resetGripper()
    pap = PickAndPlace()  
    pap.add_obstacle("table", x, y, z - .03, 1.20, 1.10, 0.1)

    while not rospy.is_shutdown():
        topLeftX, topLeftY = 0.795, -0.017

        #raw_input("Press <Enter> to pick and place: ")
        #pap.pick_and_place(x, y, z, endx, endy)

        raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        pap.move_to_position(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
        print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))

        o_x, o_y, o_z, o_w = rotations[1]
        raw_input("Press <Enter> to move the right arm to goal pose 2: ")   
        pap.move_to_position(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
        print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))
        
        o_x, o_y, o_z, o_w = rotations[2]
        raw_input("Press <Enter> to move the right arm to goal pose 3: ")   
        pap.move_to_position(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
        print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))
        
        o_x, o_y, o_z, o_w = rotations[3]
        raw_input("Press <Enter> to move the right arm to goal pose 4: ")   
        pap.move_to_position(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
        print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))

def resetGripper():
    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')
    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    #Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    #Open the right gripper
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)
    print('Done!')

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