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
from baxter_interface import Limb
from baxter_interface import Gripper
#from intera_interface import Gripper
#from intera_interface import Limb

def main():
    planner = PathPlanner("right_arm")

    #Set up the right gripper------------NOT WORKING
    right_gripper = Gripper('right')
    print(right_gripper.type())
    #Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(3.0)
    #Open the right gripper
    print('Opening...')
    right_gripper.stop()
    rospy.sleep(1.0)
    print('Done!')

    ## Add the obstacle to the planning scene here
    """
    X = 0.5, Y = 0.00, Z = 0.00
    X = 0.00, Y = 0.00, Z = 0.00, W = 1.00
    X = 0.40, Y = 1.20, Z = 0.10
    ""#
    X = 1.20
    Y = 0.10
    Z = 0.50
    box_size = np.array([X, Y, Z])
    box_name = "box"

    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    #x, y, and z position
    box_pose.pose.position.x = 0.9
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = Z / 2
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

        raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=0.4, y=-0.3, z=0.2)
                plan = planner.plan_to_pose(goal, orientation_constraints)

                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        raw_input("Press <Enter> to move the right arm to goal pose 2: ")   
        while not rospy.is_shutdown():
            try:
                goal_2 = create_target_pose(x=0.6, y=-0.3, z=0.0)
                plan = planner.plan_to_pose(goal_2, orientation_constraints)

                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        raw_input("Press <Enter> to move the right arm to goal pose 3: ")
        while not rospy.is_shutdown():
            try:
                goal_3 = create_target_pose(x=0.6, y=-0.1, z=0.1)
                plan = planner.plan_to_pose(goal_3, orientation_constraints)

                if not planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()