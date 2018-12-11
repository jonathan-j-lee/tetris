"""
pnp -- Module for performing pick-and-place tasks.
"""

from __future__ import division, generators, print_function
import numpy as np
from baxter_interface import Gripper, AnalogIO, Limb
import rospy
from env import log_pose, add_transform_offset, convert_pose, PNPEnvironment
from planner import PathPlanner
from tf.transformations import euler_from_quaternion


class SuctionPNPTask:
    """
    A representation of a generic pick-and-place task using a suction cup.

    Attributes:
        gripper_side (str): 'left' or 'right'.
        planner: The path planner.
        gripper: The gripper.
        vacuum_sensor: The vacuum sensor.
    """
    GRIP_MAX_VALUE = 175

    def __init__(self, frame_id='base', gripper_side='right'):
        self.gripper_side = gripper_side
        self.limb = Limb(gripper_side)
        self.planner = PathPlanner(frame_id, gripper_side + '_arm')
        self.calibrate_gripper()
        if rospy.get_param('verbose'):
            rospy.loginfo('Initialized PNP task.')

    def set_joint_angles(self, angles, tolerance=0.05, max_steps=1000, period=0.01):
        all_angles = dict(angles)
        for name in self.limb.joint_names():
            if name not in all_angles:
                all_angles[name] = self.limb.join_angle(name)
        step = 0
        done = lambda: all(abs(angle - self.limb.joint_angle(name)) < tolerance
                           for name, angle in all_angles.items())
        while step < max_steps and not done():
            self.limb.set_joint_positions(all_angles)
            rospy.sleep(period)
            step += 1
        return done()

    def is_grasping(self, threshold=50):
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
        if rospy.get_param('verbose'):
            rospy.loginfo('Current vacuum value: {}'.format(gripper_value))
        if gripper_value > self.GRIP_MAX_VALUE:
            raise ValueError('Detected unsafe vacuum value of {}.'.format(gripper_value))
        return gripper_value > threshold

    def calibrate_gripper(self):
        """ Initialize the gripper and its vacuum sensor. """
        self.gripper = Gripper(self.gripper_side)
        self.gripper.calibrate()
        self.vacuum_sensor = AnalogIO(self.gripper_side + '_vacuum_sensor_analog')
        if rospy.get_param('verbose'):
            rospy.loginfo('Calibrated gripper. (type={})'.format(self.gripper.type()))

    def open_gripper(self, delay=1):
        """ Open the gripper with a given delay afterwards. """
        self.gripper.open()
        rospy.sleep(delay)
        if rospy.get_param('verbose'):
            rospy.loginfo('Opened gripper.')

    def close_gripper(self, delay=1):
        """ Close a gripper with a given delay afterwards. """
        self.gripper.close()
        rospy.sleep(delay)
        if rospy.get_param('verbose'):
            rospy.loginfo('Closed gripper.')


class TetrisPNPTask(SuctionPNPTask):
    """
    A representation of the Tetris pick-and-place task.
    """
    def __init__(self, frame_id='base', gripper_side='right'):
        SuctionPNPTask.__init__(self, frame_id, gripper_side)
        self.env = PNPEnvironment(frame_id=frame_id)

    def search(self):
        angles = self.env.ARM_JOINTS_NEUTRAL[self.gripper_side]
        angles = {self.gripper_side + '_' + name: angle
                  for name, angle in zip(self.env.JOINT_NAMES, angles)}
        self.set_joint_angles(angles)

    def grasp(self, position):
        z_offset, z_delta = rospy.get_param('z_offset'), rospy.get_param('z_delta')
        z_max_steps = rospy.get_param('z_max_steps')

        current_pos = np.copy(position)
        current_pos[2] += z_offset
        steps = 0
        while not rospy.is_shutdown() and steps < z_max_steps:
            try:
                self.planner.move_to_pose(current_pos, self.env.DOWNWARDS)
            except Exception as exc:
                rospy.logerr(exc)
                break
            self.close_gripper()
            if not self.is_grasping():
                self.open_gripper()
                current_pos[2] -= z_delta
            else:
                self.env.table_height = current_pos[2] - rospy.get_param('board_thickness')
                log_pose('Grasped object.', position)
                return True
            steps += 1
        if rospy.get_param('verbose'):
            log_pose('Failed to grasp object.', position, orientation)
        return False

    def pick(self, tile_name):
        assert not self.is_grasping()
        center_pos, center_orien = self.env.find_tile_center(tile_name)
        rospy.loginfo(str(center_pos) + ' ' + str(center_orien))
        # self.planner.move_to_pose(center_pos, self.env.DOWNWARDS)
        return self.grasp(center_pos)

    def elevate(self, z_offset):
        trans = self.env.get_gripper_transform()
        offset = np.array([0, 0, z_offset])
        position, orientation = convert_pose(add_transform_offset(trans, offset))
        self.planner.move_to_pose_with_planner(position, orientation)

    def rotate_to(self, orientation):
        trans = self.env.get_gripper_transform()
        translation = trans.transform.translation
        position = np.array([translation.x, translation.y, translation.z])
        self.planner.move_to_pose(position, orientation)

    def place(self, tile):
        assert self.is_grasping()
        lift = rospy.get_param('lift_offset')
        thickness = rospy.get_param('board_thickness')

        self.elevate(lift + 2*thickness)
        # Remain downwards
        constraint = self.planner.make_orientation_constraint(self.env.DOWNWARDS,
                                                              self.env.tool_frame_id)
        position, orientation = self.env.find_slot_transform(tile)
        self.planner.move_to_pose_with_planner(position, orientation, [constraint])
        self.rotate_to(self.env.ROTATIONS[tile.rotations])
        self.elevate(-lift - thickness + rospy.get_param('drop_offset'))
        self.open_gripper()
