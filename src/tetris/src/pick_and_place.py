import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from path_planner import *
from baxter_interface import Limb, AnalogIO
from baxter_interface import gripper as robot_gripper
import baxter_interface

class PickAndPlace(object):

    def __init__(self):
        self.planner = PathPlanner("right_arm")

        #Set up the right gripper
        self.right_gripper = robot_gripper.Gripper('right')
        #Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        self.right_gripper.calibrate()
        rospy.sleep(2.0) 

        """
        #Open the right gripper
        print('Opening...')
        self.right_gripper.open()
        rospy.sleep(1.0)
        print('Dropped grasped object')
        """

    def add_obstacle(self, name, x_pos, y_pos, z_pos, x_width, y_width, z_width):
        #LENGTHS
        X = x_width
        Y = y_width
        Z = z_width
        box_size = np.array([X, Y, Z])
        box_name = "table"

        box_pose = PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.position.x = x_pos
        box_pose.pose.position.y = y_pos
        box_pose.pose.position.z = z_pos - (z_width / 2)
        #Orientation as a quaternion
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0

        self.planner.add_box_obstacle(box_size, box_name, box_pose)

        return box_name

    def remove_obstacle(self, name):
        self.planner.remove_obstacle(box_name)

    def move_to_position(self, x, y, z, orientation_constraints=list()):
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z)
                plan = self.planner.plan_to_pose(goal, orientation_constraints)

                if not self.planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

    def move_to_position_and_grasp(self, x, y, z, orientation_constraints=list()):
        """
            Returns the z coordinate (height) at which it was able to grasp the object
        """
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z)
                plan = self.planner.plan_to_pose(goal, orientation_constraints)

                if not self.planner.execute_plan(plan): 
                    raise Exception("Execution failed")

                #Close the right gripper
                print('Closing...')
                self.right_gripper.close()
                rospy.sleep(1.0)
            except Exception as e:
                print e
            else:
                if baxter_interface.AnalogIO('right_vacuum_sensor_analog').state() < 20:
                    print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
                    z -= .01
                else:
                    return z #break

    def move_to_position_and_open(self, x, y, z, orientation_constraints=list()):
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z)
                plan = self.planner.plan_to_pose(goal, orientation_constraints)

                if not self.planner.execute_plan(plan): 
                    raise Exception("Execution failed")

                #Open the right gripper
                print('Opening...')
                self.right_gripper.open()
                rospy.sleep(1.0)
                print('Dropped grasped object')
            except Exception as e:
                print e
            else:
                break

    def pick_and_place(self, from_x, from_y, z, to_x, to_y):
        z = self.move_to_position_and_grasp(from_x, from_y, z)
        print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
        self.move_to_position(from_x, from_y, z + 0.1)
        self.move_to_position_and_open(to_x, to_y, z)
