"""
pnp -- Module for performing pick-and-place tasks.
"""

import sys

import rospy
import numpy as np
import tf2_ros

from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import (
    GetPositionIK,
    GetPositionIKRequest,
    GetPositionIKResponse
)
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from tf.transformations import *

from baxter_interface import Limb, Gripper, AnalogIO

from planner import PathPlanner


class PickAndPlaceTask:
    """
    A representation of the Tetris pick-and-place task.

    Attributes:
        gripper_side (str): 'left' or 'right'.
        planner: The path planner.
        gripper: The gripper.
        vacuum_sensor: The vacuum sensor.
        table_height (float): The estimated z-coordinate of the table surface.
    """
    GRIP_MAX_VALUE = 175
    Z_COORD_DELTA = 0.01

    def __init__(self, gripper_side='right'):
        self.gripper_side = gripper_side
        self.planner = PathPlanner('base', gripper_side + '_arm', verbose=True)
        self.table_height = np.nan
        self.calibrate_gripper()

    def is_grasping(self, threshold=30):
        """
        Uses the vacuum sensor to detect whether the suction cup has grasped an
        object. The topics

            /robot/analog_io/left_vacuum_sensor_analog/value_uint32
            /robot/analog_io/left_vacuum_sensor_analog/state

        both give the direct analog readings from the vacuum sensor in the
        gripper. According to our gripper engineer, the values mean:

            0 - 46:   The vacuum gripper is likely not attached to an object.
            47 - 175: The vacuum is likely attached to an object (usually
                      around 150 when grasping).
            176+:     There is likely a short between 5V and signal on the sensor.

        Arguments:
            threshold (int): Values above this threshold constitute a grip.
        """
        gripper_value = self.vacuum_sensor.state()
        rospy.loginfo('Current vacuum value: {}'.format(gripper_value))
        if gripper_value > self.GRIP_MAX_VALUE:
            message = 'Detected unsafe vacuum value of {}.'.format(gripper_value)
            rospy.logerr(message)
            raise ValueError(message)
        return gripper_value > threshold

    def calibrate_gripper(self):
        """ Initialize the gripper and its vacuum sensor. """
        self.gripper = Gripper(self.gripper_side)
        self.gripper.calibrate()
        self.vacuum_sensor = AnalogIO(self.gripper_side + '_vacuum_sensor_analog')
        rospy.loginfo('Calibrated gripper. (type={})'.format(self.gripper.type()))

    def open_gripper(self, delay=1):
        """ Open the gripper with a given delay afterwards. """
        self.gripper.open()
        rospy.sleep(delay)
        rospy.loginfo('Opened gripper.')

    def close_gripper(self, delay=1):
        """ Close a gripper with a given delay afterwards. """
        self.gripper.close()
        rospy.sleep(delay)
        rospy.loginfo('Closed gripper.')


"""
    r2 = np.sqrt(2)/2
    rotations = np.array([
        [0, 1, 0, 0],
        [r2, r2, 0, 0],
        [1, 0, 0, 0],
        [r2, -r2, 0, 0],
    ])
    ROT_0       = 0
    ROT_90      = 1
    ROT_180     = 2
    ROT_270     = 3



    def findTableHeight(self, approx_x, approx_y, approx_z):
        rot = self.rotations[0]
        self.table_height = self.move_to_position_and_grasp(approx_x, approx_y, approx_z, o_x=rot[0], o_y=rot[1], o_z=rot[2], o_w=rot[3])
        self.right_gripper.open()

    def add_obstacle(self, name, x_pos, y_pos, z_pos, x_width, y_width, z_width):
        #LENGTHS
        X = x_width
        Y = y_width
        Z = z_width
        box_size = np.array([X, Y, Z])

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base'
        box_pose.pose.position.x = x_pos
        box_pose.pose.position.y = y_pos
        box_pose.pose.position.z = z_pos - (z_width / 2)
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0
        self.planner.add_box_obstacle(box_size, name, box_pose)
        return name

    def move_and_grasp(self, x, y, z, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        # Returns the z coordinate (height) at which it was able to grasp the object
        curr_z = z
        while not rospy.is_shutdown():
            success = self.move_to_pose(x, y, curr_z, o_x, o_y, o_z, o_w)

            print("Success: {}".format(success))
            if success:
                self.closeGripper()
                if not self.isGraspingObject():
                    self.openGripper()
                    curr_z -= .001 #lower and try again
                else:
                    print("Object grasped")
                    self.table_height = curr_z
                    return self.table_height
            else:
                print("could not move to target pose")
                return -1

    def move_and_release(self, x, y, z, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        self.move_to_pose(x, y, z, o_x, o_y, o_z, o_w)
        self.open_gripper()
        rospy.loginfo('Dropped grasped object at:')
        self.log_pose(x, y, z, o_x, o_y, o_z, o_w)

    def move_to_rotation(self, x, y, z, rotation):
        # rotation = index of orientation as defined in rotations class variable above
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def move_to_rotation_and_grasp(self, x, y, z, rotation):
            rotation = index of orientation as defined in rotations class variable above
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose_and_grasp(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def move_to_rotation_and_open(self, x, y, z, rotation):
            rotation = index of orientation as defined in rotations class variable above
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose_and_open(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def pick_and_place(self, from_x, from_y, z, to_x, to_y):
        self.move_to_pose_and_grasp(from_x, from_y, z)
        self.move_to_pose(from_x, from_y, self.table_height + 0.1)
        self.move_to_pose(to_x, to_y, self.table_height + 0.1)
        self.move_to_pose_and_open(to_x - 0.003, to_y - 0.003, self.table_height)

    def rotateTo(self, rotation):
            Rotation = orientation for gripper to be in
                0: 0 position (gripper pointed straight down)
                1: 90 CW from 0
                2: 180 CW from 0
                3: 270 CW from 0

            These rotations are defined as quaternions at the beginning of this class

            Rotates the gripper to the desired rotation configuration
        translation = self.getCurrPosition()
        x, y, z = translation.x, translation.y, translation.z  #keep translational x, y, z coordinates the same
        o_x, o_y, o_z, o_w = rotations[rotation] #get proper orientation
        self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def rotateBy(self, rad):
            Rotates by rad radians about the Z-axis CW
        curr_rot = self.getCurrRotation()

        q_rot = quaternion_from_euler(0, 0, -rad)
        q_curr = (curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w)
        q_new = quaternion_multiply(q_rot, q_curr)

        translation = self.getCurrPosition()
        x, y, z = translation.x, translation.y, translation.z  #keep translational x, y, z coordinates the same
        o_x, o_y, o_z, o_w = q_new[0], q_new[1], q_new[2], q_new[3]
        self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def goLowerBy(self, offset_z):
            Gets current position of gripper and lowers it by offset_z (offset_z = positive amount that you want to lower by)
        trans = self.getCurrPosition()
        self.move_to_position(trans.x, trans.y, trans.z - offset_z)

    def getTransformToRightGripper(self):
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
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        # Loop until the node is killed with Ctrl-C
        while True:
            try:
                trans = tfBuffer.lookup_transform("base", "reference/right_gripper", rospy.Time())
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
            target_rotation: [x, y, z, w]
                the desired rotation

            Returns whether the gripper is in the desired rotation configuration
        rot = self.getCurrRotation()
        curr_rotation_list = np.array([rot.x, rot.y, rot.z, rot.w])
        error_tolerance = 0.003 * 4

        error = min(np.sum(np.abs(-curr_rotation_list - np.array(target_rotation))), np.sum(np.abs(curr_rotation_list - np.array(target_rotation))))
        print("Error: {}".format(error))
        return error < error_tolerance

    def isInDesiredPosition(self, target_position):
            target_position: [x, y, z]
                the desired position

            Returns whether the gripper is in the desired position
        pos = self.getCurrPosition()
        curr_pos_list = np.array([pos.x, pos.y, pos.z])
        error_tolerance = 0.002 * 3

        error = np.sum(np.abs(curr_pos_list - np.array(target_position)))
        print("Error: {}".format(error))
        return error < error_tolerance
"""
