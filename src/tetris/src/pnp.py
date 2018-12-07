"""
pnp -- Module for performing pick-and-place tasks.
"""

from __future__ import division, generators, print_function, unicode_literals

from baxter_interface import Limb, Gripper, AnalogIO
import numpy as np
import rospy

from env import PNPEnvironment
from planner import PathPlanner


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
        self.planner = PathPlanner(frame_id, gripper_side + '_arm')
        self.calibrate_gripper()

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
        super(TetrisPNPTask, self).__init__(frame_id, gripper_side)
        self.env = PNPEnvironment(frame_id=frame_id)

    def grasp(self, position, orientation=None):
        z_offset, z_delta = rospy.get_param('z_offset'), rospy.get_param('z_delta')
        z_max_steps = rospy.get_param('z_max_steps')

        current_pos = np.copy(position)
        current_pos[2] += z_offset
        steps = 0
        while not rospy.is_shutdown() and steps < z_max_steps:
            success = self.planner.move_to_pose(current_pos, orientation)
            if not success:
                break
            self.close_gripper()
            if not self.is_grasping():
                self.open_gripper()
                current_pos[2] -= z_delta
            else:
                self.env.table_height = current_pos[2] - rospy.get_param('board_thickness')
                self.planner.log_pose('Grasped object.', position, orientation)
                return True
            steps += 1
        if self.verbose:
            self.planner.log_pose('Failed to grasp object.', position, orientation)
        return False

    def pick(self, tile_name):
        try:
            position, orientation = self.env.find_tile_center(tile_name)
        except ValueError:
            return False
        return self.grasp(position, orientation)
