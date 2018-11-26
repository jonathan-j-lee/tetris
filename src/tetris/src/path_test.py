#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint

from path_planner import *
from SuctionGripper import *
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper
#from baxter_interface import Gripper
#from intera_interface import Gripper
#from intera_interface import Limb

def main():
    planner = PathPlanner("right_arm")

    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')
    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)    

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
    """
    
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

    while not rospy.is_shutdown():
        x, y, z = 0.816, -0.403, -0.170
        raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        while not rospy.is_shutdown():
            try:
                #0.816, -0.403, -0.169
                goal = create_target_pose(x=x, y=y, z=z)
                plan = planner.plan_to_pose(goal, orientation_constraints)

                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")

                #Close the right gripper
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)
            except Exception as e:
                print e
            else:
                if baxter_interface.AnalogIO('right_vacuum_sensor_analog').state() < 20:
                    print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
                    z -= .01
                else:
                    break

        print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
        raw_input("Press <Enter> to move the right arm to goal pose 2: ")   
        while not rospy.is_shutdown():
            try:
                goal_2 = create_target_pose(x=0.6, y=-0.3, z=0.0)
                plan = planner.plan_to_pose(goal_2, orientation_constraints)
                print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        raw_input("Press <Enter> to move the right arm to goal pose 3: ")
        while not rospy.is_shutdown():
            try:
                goal_3 = create_target_pose(x=x, y=y, z=z)
                plan = planner.plan_to_pose(goal_3, orientation_constraints)

                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")

                #Open the right gripper
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)
                print('Done!')
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()