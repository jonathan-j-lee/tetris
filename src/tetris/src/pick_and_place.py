import sys
import rospy
import numpy as np

import tf2_ros 
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import TransformStamped
from path_planner import *
from baxter_interface import Limb, AnalogIO
from baxter_interface import gripper as robot_gripper
import baxter_interface

class PickAndPlace(object):

    r2 = np.sqrt(2) / 2
    rotations = [
        (0,     1.0,     0,      0),
        (r2,    r2,     0,      0),
        (1.0,   0.0,     0,      0),
        (r2,    -r2,    0,      0)
    ]

    def __init__(self):
        self.planner = PathPlanner("right_arm")

        #Set up the right gripper
        self.right_gripper = robot_gripper.Gripper('right')
        #Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        self.right_gripper.calibrate()

        """
        self.right_gripper.close()
        rospy.sleep(1.0)
        self.right_gripper.open()
        rospy.sleep(1.0)
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

    def move_to_position(self, x, y, z, orientation_constraints=list(), o_x=0.0, o_y=-1.0, o_z=0.0, o_w=0.0):
        print("moving to position: {}, {}, {}".format(x, y, z))
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
                plan = self.planner.plan_to_pose(goal, orientation_constraints)

                if not self.planner.execute_plan(plan): 
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                print("new o_y: {}".format(o_y))
            else:
                break
        
    def move_to_position_and_grasp(self, x, y, z, orientation_constraints=list(), o_x=0.0, o_y=-1.0, o_z=0.0, o_w=0.0):
        """
            Returns the z coordinate (height) at which it was able to grasp the object
        """
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
                plan = self.planner.plan_to_pose(goal, orientation_constraints)

                print("moving to position: {}, {}, {} AND GRASPING".format(x, y, z))
                if not self.planner.execute_plan(plan): 
                    raise Exception("Execution failed")

                #Close the right gripper
                print('Closing...')
                self.right_gripper.close()
                rospy.sleep(1.0)
            except Exception as e:
                print("---------------Error in move and grasp")
                print e
            else:
                if baxter_interface.AnalogIO('right_vacuum_sensor_analog').state() < 20:
                    print("-------------Curr vacuum state:", baxter_interface.AnalogIO('right_vacuum_sensor_analog').state())
                    z -= .001
                else:
                    print("Object grasped!")
                    return z #break

    def move_to_position_and_open(self, x, y, z, orientation_constraints=list(), o_x=0.0, o_y=-1.0, o_z=0.0, o_w=0.0):
        while not rospy.is_shutdown():
            try:
                goal = create_target_pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
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
        self.move_to_position(to_x, to_y, z + 0.1)
        rospy.sleep(1.0)
        self.move_to_position_and_open(to_x - 0.003, to_y - 0.003, z)

    def getTransformToRightGripper(self):
        """
            header: 
              seq: 0
              stamp: 
                secs: 1543610434
                nsecs: 287521601
              frame_id: reference/right_gripper
            child_frame_id: base
            transform: 
              translation: 
                x: -0.244178202193
                y: 0.863870376621
                z: -0.037820321536
              rotation: 
                x: 0.707696582217
                y: -0.706514390425
                z: -0.000749940922453
                w: -0.00154959016305
        """
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        # Loop until the node is killed with Ctrl-C
        while True:
            try:
                trans = tfBuffer.lookup_transform("reference/right_gripper", "base", rospy.Time())
                #print(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            else:
                break
        return trans

    def getCurrPosition(self):
        trans = self.getTransformToRightGripper()
        return trans.transform.translation

    def getCurrRotation(self):
        trans = self.getTransformToRightGripper()
        return trans.transform.rotation

    def isInDesiredRotation(self, target_rotation):
        """
            target_rotation: [x, y, z, w]
                the desired rotation

            Returns whether the gripper is in the desired rotation configuration
        """
        rot = self.getCurrRotation()
        curr_rotation_list = np.array([rot.x, rot.y, rot.z, rot.w])
        error_tolerance = 0.003 * 4
        print(curr_rotation_list)
        print(np.array(target_rotation))
        error = min(np.sum(np.abs(-curr_rotation_list - np.array(target_rotation))), np.sum(np.abs(curr_rotation_list - np.array(target_rotation))))
        return error

    def rotateTo(self, rotation):
        """
            Rotation = orientation for gripper to be in
                0: 0 position (gripper pointed straight down)
                1: 90 CW from 0
                2: 180 CW from 0
                3: 270 CW from 0

            These rotations are defined as quaternions at the beginning of this class

            Rotates the gripper to the desired rotation configuration
        """
        translation = self.getCurrPosition()
        x, y, z = translation.x, translation.y, translation.z  #keep translational x, y, z coordinates the same
        o_x, o_y, o_z, o_w = rotations[rotation] #get proper orientation

        self.move_to_position(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)