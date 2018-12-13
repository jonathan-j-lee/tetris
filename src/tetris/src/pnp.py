"""
pnp -- Module for performing pick-and-place tasks.
"""

from __future__ import division, generators, print_function
import numpy as np
from baxter_interface import Gripper, AnalogIO, Limb
import rospy
from env import log_pose, marker_frame, add_transform_offset, convert_pose, PNPEnvironment
from planner import PathPlanner
from solver import TILE_TYPES
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
        assert gripper_side in ('left', 'right')
        self.gripper_side = gripper_side
        self.camera_side = 'left' if self.gripper_side == 'right' else 'right'
        self.gripper_planner = PathPlanner(frame_id, self.gripper_side + '_arm')
        self.camera_planner = PathPlanner(frame_id, self.camera_side + '_arm')
        self.calibrate_gripper()
        if rospy.get_param('verbose'):
            rospy.loginfo('Initialized PNP task.')

    def set_joint_angles(self, angles, limb, tolerance=0.05, max_steps=1000, period=0.01):
        all_angles = dict(angles)
        for name in limb.joint_names():
            if name not in all_angles:
                all_angles[name] = limb.join_angle(name)

        step = 0
        done = lambda: all(abs(angle - limb.joint_angle(name)) < tolerance
                           for name, angle in all_angles.items())
        while step < max_steps and not done():
            limb.set_joint_positions(all_angles)
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
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.open_gripper()
        if self.env.table_placed:
            self.gripper_planner.remove_obstacle(self.env.TABLE_OBSTACLE_NAME)

    def search(self, frames, delay=2):
        # TODO: try to get multiple transforms to get a better estimate
        self.gripper_planner.move_to_pose(
            self.env.NEUTRAL_POSITIONS[self.gripper_side], self.env.DOWNWARDS)
        frame_transforms, missing_frames = {}, []
        for position in self.env.SEARCH_POSITIONS:
            self.camera_planner.move_to_pose_with_planner(position, self.env.DOWNWARDS)
            rospy.sleep(delay)  # Allow frames to stabilize.
            for frame in frames:
                if frame not in frame_transforms:
                    trans = self.env.get_rel_transform(frame)
                    if trans is not None:
                        frame_transforms[frame] = trans
            if all(frame in frame_transforms for frame in frames):
                break
        else:
            for frame in frames:
                if frame not in frame_transforms:
                    missing_frames.append(frame)
            if rospy.get_param('verbose'):
                rospy.logwarn('Unable to find some frames: ' + ', '.join(missing_frames))

        self.camera_planner.move_to_pose_with_planner(
            self.env.NEUTRAL_POSITIONS[self.camera_side], self.env.DOWNWARDS)
        return frame_transforms, missing_frames

    def grasp(self, position, orientation):
        z_offset, z_delta = rospy.get_param('z_offset'), rospy.get_param('z_delta')
        z_max_steps = rospy.get_param('z_max_steps')

        current_pos = np.copy(position)
        current_pos[2] += z_offset
        self.gripper_planner.move_to_pose_with_planner(current_pos, orientation)
        if not rospy.get_param('grasp_enabled'):
            return

        steps = 0
        while not rospy.is_shutdown() and steps < z_max_steps:
            self.close_gripper()
            if not self.is_grasping():
                self.open_gripper()
                current_pos[2] -= z_delta
                self.gripper_planner.move_to_pose_with_planner(current_pos, orientation)
            else:
                self.env.table_height = current_pos[2] - rospy.get_param('board_thickness')
                log_pose('Grasped object.', position)
                break
            steps += 1
        else:
            if rospy.get_param('verbose'):
                log_pose('Exceeded number of steps during grasp.', position, orientation)

    def pick(self, tile_name):
        assert not self.is_grasping()
        board_id = marker_frame(rospy.get_param('board_top_left_marker'))
        tile_type = TILE_TYPES[tile_name]
        tile_id = marker_frame(tile_type.marker_id)
        frame_transforms, missing_frames = self.search([board_id, tile_id])
        if not self.env.table_placed and board_id in frame_transforms:
            self.env.place_table(self.gripper_planner, frame_transforms[board_id])
        if missing_frames:
            raise ValueError('Unable to find all transforms. Retry.')
        if rospy.get_param('verbose'):
            for name, trans in frame_transforms.items():
                position, orientation = convert_pose(add_transform_offset(trans))
                log_pose('Frame "{}"'.format(name), position, orientation)

        center_pos, center_orien = self.env.find_tile_center(
            tile_type, frame_transforms[tile_id])
        self.grasp(center_pos, center_orien)
        return frame_transforms[board_id], frame_transforms[tile_id]

    def rotate_to(self, orientation):
        trans = self.env.get_gripper_transform()
        translation = trans.transform.translation
        position = np.array([translation.x, translation.y, translation.z])
        self.gripper_planner.move_to_pose(position, orientation)

    def place(self, tile, board_trans, tile_trans):
        if rospy.get_param('grasp_enabled'):
            assert self.is_grasping()
        lift = rospy.get_param('lift_offset')
        thickness = rospy.get_param('board_thickness')
        position, orientation = convert_pose(add_transform_offset(tile_trans))
        position[2] += lift + 2*thickness
        self.gripper_planner.move_to_pose_with_planner(position, orientation)

        # TODO: test
        orientation = self.env.ROTATIONS[tile.rotations]
        self.rotate_to(orientation)
        position = self.env.find_slot_transform(tile, board_trans, position[2])
        self.gripper_planner.move_to_pose_with_planner(position, orientation)
        position[2] -= lift + thickness
        self.gripper_planner.move_to_pose_with_planner(position, orientation)
        self.open_gripper()
