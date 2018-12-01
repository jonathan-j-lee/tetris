#!/usr/bin/env python
"""
Path Planner Class for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer

    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene


    """
    def __init__(self, group_name):
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """

        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # This publishes updates to the planning scene
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(5)

        # Set the bounds of the workspace
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def move_to_position(self, target_position):
        """
            target_position: [x, y, z]
        """
        try:
            self._group.set_position_target(target_position)
            self._group.go()
        except rospy.ServiceException, e:
            print("move_to_position({}, {}, {}) failed".format(x, y, z))
            print(e)

    def move_to_pose(self, x, y, z, o_x, o_y, o_z, o_w):
        try:
            goal = create_target_pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
            self._group.set_pose_target(goal)
            self._group.go()
        except Exception as e:
            print("move_to_pose({}, {}, {}, o_x={}, o_y={}, o_z={}, o_w={}) failed".format(x, y, z, o_x, o_y, o_z, o_w))
            print(e)
            return False
        return True

    def plan_to_pose(self, target, orientation_constraints):
        """
        Generates a plan given an end effector pose subject to orientation constraints

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """
        self._group.set_pose_target(target)
        self._group.set_start_state_to_current_state()

        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self._group.set_path_constraints(constraints)

        plan = self._group.plan()

        return plan

    def execute_plan(self, plan):
        """
        Uses the robot's built-in controllers to execute a plan

        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """

        return self._group.execute(plan, True)


    def add_box_obstacle(self, size, name, pose):
        """
        Adds a rectangular prism obstacle to the planning scene

        Inputs:
        size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
        name: unique name of the obstacle (used for adding and removing)
        pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
        """    

        # Create a CollisionObject, which will be added to the planning scene
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        # Create a box primitive, which will be inside the CollisionObject
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size

        # Fill the collision object with primitive(s)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]

        # Publish the object
        self._planning_scene_publisher.publish(co)

    def remove_obstacle(self, name):
        """
        Removes an obstacle from the planning scene

        Inputs:
        name: unique name of the obstacle
        """

        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name

        self._planning_scene_publisher.publish(co)

def create_target_position(x, y, z):
    goal = PoseStamped()
    goal.header.frame_id = 'base'

    #x, y, and z position
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    return goal 

def create_target_pose(x=0.0, y=0.0, z=0.0, o_x=0.0, o_y=-1.0, o_z=0.0, o_w=0.0):
    """
        Default to down orientation (0, -1, 0, 0)
    """
    goal = create_target_position(x, y, z)

    #Orientation as a quaternion
    goal.pose.orientation.x = o_x
    goal.pose.orientation.y = o_y
    goal.pose.orientation.z = o_z
    goal.pose.orientation.w = o_w
    return goal 