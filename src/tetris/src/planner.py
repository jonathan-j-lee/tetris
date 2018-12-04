"""
planner -- Module for performing path planning.
"""

from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
import numpy as np
import rospy
from shape_msgs.msg import SolidPrimitive


class PathPlanner:
    """
    This path planner wraps the planning and actuation functionality provided by MoveIt.

    All positions are arrays holding x, y, and z coordinates. Orientations are
    arrays holding x, y, z, and w coordinates (in quaternion form).

    Attributes:
        frame_id (str): The frame all coordinates are relative to.
        workspace (list): The bounds of the planning space (a box). Specified
            as the minimum x, y, and z coordinates, then the maximum x, y, and
            z coordinates.
        group_name (str): The MoveIt group name (for example, 'right_arm').
        time_limit (float): The maximum number of seconds MoveIt will plan for.
        verbose (bool): If true, the planner will log messages to ROS.
        robot: The MoveIt robot commander.
        scene: The planning scene.
        group: The MoveIt MoveGroup.
        scene_publisher: A publisher that updates the planning scene.
    """
    DEFAULT_POSITION = np.zeros(3)
    DEFAULT_ORIENTATION = np.array([0, -1, 0, 0])
    PLANNING_SCENE_TOPIC = '/collision_object'
    CONSTRAINT_TYPES = {
        'orientation': 0,
    }

    def __init__(self, frame_id, group_name, time_limit=5, workspace=None, verbose=False):
        if not workspace:
            workspace = [-2, -2, -2, 2, 2, 2]
        self.frame_id, self.workspace = frame_id, workspace
        self.group_name = group_name
        self.time_limit, self.verbose = time_limit, verbose
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander(group_name)
        self.scene_publisher = rospy.Publisher(self.PLANNING_SCENE_TOPIC,
                                               CollisionObject, queue_size=10)

        rospy.on_shutdown(self.shutdown)
        self.group.set_planning_time(time_limit)
        self.group.set_workspace(workspace)
        rospy.sleep(0.5)  # Sleep to ensure initialization has finished.
        if verbose:
            rospy.info('Initialized path planner.')

    def shutdown(self):
        """ Stop the path planner. """
        del self.group
        if self.verbose:
            rospy.logwarn('Terminated path planner.')

    def create_target_pose(self, position=None, orientation=None):
        """
        Convert a pose as arrays into a ROS-compatible timestamped pose data type.

        Arguments:
            position: The pose position. Defaults to the `frame_id` origin.
            orientation: The pose orientation. Defaults to the "down" orientation.
        """
        if not position:
            position = self.DEFAULT_POSITION
        if not orientation:
            orientation = self.DEFAULT_ORIENTATION
        target = PoseStamped()
        target.header.frame_id = self.frame_id
        target_pos, target_orien = target.pose.position, target.pose.orientation
        target_pos.x, target_pos.y, target_pos.z = position
        target_orien.x, target_orien.y, target_orien.z, target_orien.w = orientation
        return target

    def log_pose(self, msg, position, orientation=None):
        """ Logs the given pose. """
        if not orientation:
            rospy.info('{} (x={}, y={}, z={})'.format(msg, *position))
        else:
            template = '{} (x={}, y={}, z={}, o_x={}, o_y={}, o_z={}, o_w={})'
            rospy.info(template.format(msg, *position, *orientation))

    def move_to_pose(self, position, orientation=None):
        """
        Move the end effector to the given pose naively.

        Arguments:
            position: The x, y, and z coordinates to move to.
            orientation: The orientation to take (quaternion, optional).

        Returns (bool): Whether or not the movement executed successfully.
        """
        if not orientation:
            self.group.set_position_target(position)
        else:
            self.group.set_pose_target(self.create_target_pose(position, orientation))
        if self.verbose:
            self.log_pose('Moving to pose.', position, orientation)
        try:
            self.group.go()
        except rospy.ServiceException:
            return False
        else:
            return True

    def move_to_pose_with_planner(self, position, orientation=None, orientation_constraints=None):
        """
        Move the end effector to the given pose, taking into account
        constraints and planning scene obstacles.

        Arguments:
            position: The x, y, and z coordinates to move to.
            orientation: The orientation to take (quaternion, optional).
            orientation_constraints (list): A list of `OrientationConstraint`
                objects created with `make_orientation_constraint`.

        Returns (bool): Whether or not the movement executed successfully.
        """
        if not orientation_constraints:
            orientation_constraints = []
        target = self.create_target_pose(position, orientation)
        if self.verbose:
            self.log_pose('Moving to pose with planner.', position, orientation)
        try:
            plan = self.plan_to_pose(target, orientation_constraints)
        except:  # FIXME: find and use a more specific exception
            rospy.logwarn('Failed to generate plan.')
            return False
        return self.group.execute(plan, True)

    def plan_to_pose(self, target, orientation_constraints):
        """
        Plan a movement to a pose from the current state, given constraints.

        Arguments:
            target: The destination pose.
            orientation_constraints (list): The constraints.

        Returns (moveit_msgs.msg.RobotTrajectory): The path.
        """
        self.group.set_pose_target(target)
        self.group.set_start_state_to_current_state()
        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self.group.set_path_constraints(constraints)
        return self.group.plan()

    def add_box_obstacle(self, dimensions, name, com_position, com_orientation):
        """
        Add a rectangular prism obstacle to the planning scene.

        Arguments:
            dimensions: An array containing the width, length, and height of
                the box (in the box's body frame, corresponding to x, y, and z).
            name: A unique name for identifying this obstacle.
            com_position: The position of the center-of-mass (COM) of this box,
                relative to the global frame `frame_id`.
            com_orientation: The orientation of the COM.
        """
        obj = CollisionObject()
        obj.id, obj.operation, obj.header = name, CollisionObject.ADD, pose.header
        box = SolidPrimitive()
        box.type, box.dimensions = SolidPrimitive.BOX, dimensions
        obj.primitives, obj.primitive_poses = [box], [pose.pose]
        self.scene_publisher.publish(obj)
        if self.verbose:
            rospy.loginfo('Added box object "{}" to planning scene: '
                          '(x={}, y={}, z={}).'.format(name, *dimensions))

    def remove_obstacle(self, name):
        """
        Remove a named obstacle from the planning scene.

        Arguments:
            name: The name of the obstacle, as provided during its addition.
        """
        obj = CollisionObject()
        obj.id, obj.operation = name, CollisionObject.REMOVE
        self.scene_publisher.publish(obj)
        if self.verbose:
            rospy.loginfo('Removed object "{}" from planning scene.'.format(name))

    def make_orientation_constraint(self, orientation, link_id):
        """
        Make an orientation constraint in the context of the robot and world.

        Arguments:
            orientation: The orientation the link should have.
            link_id: The name of the link frame.

        Returns (OrientationConstraint): The constraint.
        """
        constraint = OrientationConstraint()
        constraint.header.frame_id = self.frame_id
        constraint.link_name = link_id
        const_orien = constraint.orienation
        const_orien.x, const_orien.y, const_orien.z, const_orien.w = orientation
        return constraint
