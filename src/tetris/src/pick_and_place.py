import sys
import rospy
import numpy as np
import tf2_ros 
import tf2_geometry_msgs

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from tf.transformations import *

from path_planner import *
from TetrisSolver import *
from Pieces import *
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
    ROT_0       = 0
    ROT_90      = 1
    ROT_180     = 2
    ROT_270     = 3

    def __init__(self, z_offset=0):
        self.planner = PathPlanner("right_arm")

        self.table_height = -1
        self.z_offset = z_offset

        self.initGripper()
        self.runTetrisSolver()

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
        box_pose.header.frame_id = "base"
        box_pose.pose.position.x = x_pos
        box_pose.pose.position.y = y_pos
        box_pose.pose.position.z = z_pos + self.z_offset - (z_width / 2)
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0
        self.planner.add_box_obstacle(box_size, name, box_pose)
        return name

    def remove_obstacle(self, name):
        self.planner.remove_obstacle(name)

###################Gripper stuff

    def initGripper(self):
        #Set up the right gripper
        self.right_gripper = robot_gripper.Gripper('right')
        print('Calibrating gripper...')
        self.right_gripper.calibrate()

    def closeGripper(self):
        print("Closing gripper...")
        self.right_gripper.close()
        rospy.sleep(1.0)
    
    def openGripper(self):
        print('Opening gripper...')
        self.right_gripper.open()
        rospy.sleep(1.0)

    def isGraspingObject(self):
        """
        Both the topics /robot/analog_io/left_vacuum_sensor_analog/value_uint32 and /robot/analog_io/left_vacuum_sensor_analog/state 
        give you the direct analog readings from the vacuum sensor in the vacuum gripper. According to our gripper engineer, the values mean:

        Value       Meaning
        0-46 :      The vacuum gripper is likely not attached to an object
        47-175:     The vacuum is likely attached to an object (usually around 150 when grasping)
        175+:       There is likely a short between 5V and signal on the sensor.
        """
        threshold = 30
        print("-------------Curr vacuum state: {}".format(baxter_interface.AnalogIO('right_vacuum_sensor_analog').state()))
        return baxter_interface.AnalogIO('right_vacuum_sensor_analog').state() > threshold

######################Pick and place stuff

    def move_to_position(self, x, y, z):
        self.planner.move_to_position(x, y, z + self.z_offset)

    def move_to_pose(self, x, y, z, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        return self.planner.move_to_pose_planner(x, y, z + self.z_offset, o_x, o_y, o_z, o_w)

    def move_to_pose_on_table(self, x, y, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        if self.table_height == -1:
            print("FORGOT TO SET TABLE HEIGHT")
            return False
        return self.move_to_pose(x, y, self.table_height + 0.008, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def move_to_pose_on_board(self, x, y, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        if self.table_height == -1:
            print("FORGOT TO SET TABLE HEIGHT")
            return False
        return self.move_to_pose(x, y, self.table_height + Piece.boardThickness, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
        
    def move_to_pose_and_grasp(self, x, y, z, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        """
            Returns the z coordinate (height) at which it was able to grasp the object
        """
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

    def move_to_pose_and_open(self, x, y, z, o_x=0.0, o_y=1.0, o_z=0.0, o_w=0.0):
        self.move_to_pose(x, y, z, o_x, o_y, o_z, o_w)
        self.openGripper()
        print("Dropped any grasped object")

    def move_to_rotation(self, x, y, z, rotation):
        """
            rotation = index of orientation as defined in rotations class variable above
        """
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def move_to_rotation_and_grasp(self, x, y, z, rotation):
        """
            rotation = index of orientation as defined in rotations class variable above
        """
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose_and_grasp(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def move_to_rotation_and_open(self, x, y, z, rotation):
        """
            rotation = index of orientation as defined in rotations class variable above
        """
        o_x, o_y, o_z, o_w = self.rotations[rotation]
        return self.move_to_pose_and_open(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def pick_and_place(self, from_x, from_y, z, to_x, to_y):
        self.move_to_pose_and_grasp(from_x, from_y, z)
        self.move_to_pose(from_x, from_y, self.table_height + 0.1)
        self.move_to_pose(to_x, to_y, self.table_height + 0.1)
        self.move_to_pose_and_open(to_x - 0.003, to_y - 0.003, self.table_height)

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
        self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def rotateBy(self, rad):
        """
            Rotates by rad radians about the Z-axis CW
        """
        curr_rot = self.getCurrRotation()

        q_rot = quaternion_from_euler(0, 0, -rad)
        q_curr = (curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w)
        q_new = quaternion_multiply(q_rot, q_curr)

        translation = self.getCurrPosition()
        x, y, z = translation.x, translation.y, translation.z  #keep translational x, y, z coordinates the same
        o_x, o_y, o_z, o_w = q_new[0], q_new[1], q_new[2], q_new[3]
        self.move_to_pose(x, y, z, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)

    def goLowerBy(self, offset_z):
        """
            Gets current position of gripper and lowers it by offset_z (offset_z = positive amount that you want to lower by)
        """
        trans = self.getCurrPosition()
        self.move_to_position(trans.x, trans.y, trans.z - offset_z)

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
        """
            target_rotation: [x, y, z, w]
                the desired rotation

            Returns whether the gripper is in the desired rotation configuration
        """
        rot = self.getCurrRotation()
        curr_rotation_list = np.array([rot.x, rot.y, rot.z, rot.w])
        error_tolerance = 0.003 * 4

        error = min(np.sum(np.abs(-curr_rotation_list - np.array(target_rotation))), np.sum(np.abs(curr_rotation_list - np.array(target_rotation))))
        print("Error: {}".format(error))
        return error < error_tolerance

    def isInDesiredPosition(self, target_position):
        """
            target_position: [x, y, z]
                the desired position

            Returns whether the gripper is in the desired position
        """
        pos = self.getCurrPosition()
        curr_pos_list = np.array([pos.x, pos.y, pos.z])
        error_tolerance = 0.002 * 3

        error = np.sum(np.abs(curr_pos_list - np.array(target_position)))
        print("Error: {}".format(error))
        return error < error_tolerance

####################################Tetris Solver stuff

    def runTetrisSolver(self):
        self.solver = OurSolver(
            boardRows = 6,
            boardCols = 8,
            numTiles = [2, 2, 2, 2, 1, 2, 1]
        )
        self.solver.solve()
        """
        for piece in self.solver.getOrderForPlacement():
            print("Piece: ", self.solver.tileTypes[piece.tile_index].__name__)
            print("\tR: {}, C: {}, rot: {}".format(piece.row, piece.col, piece.rotation))
            print("\tCoordinates: {}".format(self.solver.getCoordinatesForARTagOfPiece(piece)))
            #print("\tPose for piece: ", self.getBasePoseForPiece(piece))
        """

    def testPoseForPiece(self):
        pass
        #print("Transform from board: ", self.getTransformFromBoard())

####################################Place on board code
    def getTransformFromARTag(self, ar_marker_id):
        """
            ar_marker_id: e.g. 'ar_marker_7'
        """
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        print("getting transfrom from {} to base".format(ar_marker_id))
        while True:
            try:
                trans = tfBuffer.lookup_transform("base", ar_marker_id, rospy.Time())
                #print(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            else:
                break
        return trans  

    def move_to_ar_marker(self, ar_marker_id):
        #self.solver.getTileType(piece).ar_marker_id
        transform = self.getTransformFromARTag(ar_marker_id).transform
        print(transform)
        x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
        o_x, o_y, o_z, o_w = transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
        e_x, e_y, e_z = euler_from_quaternion([o_x, o_y, o_z, o_w])

        #only care about the rotation about z axis
        new_ox, new_oy, new_oz, new_ow = quaternion_from_euler(0, 0, e_z - (np.pi / 2))
        print(new_ox, new_oy, new_oz, new_ow)

        if self.table_height == -1:
            print("FORGOT TO SET TABLE HEIGHT")
        self.move_to_pose(x, y, self.table_height + 0.008, o_x=new_ow, o_y=new_oz, o_z=0.0, o_w=0.0)

    def move_to_piece_CoM(self, piece):
        #Pose of the Center of Mass w.r.t. the piece's AR tag
        tileType = self.solver.getTileType(piece)
        print("TileType: {}, ar_marker_id: {}".format(tileType.__name__, tileType.ar_marker_id))
        piece_CoM_pose_stamped = PoseStamped()
        piece_CoM_pose_stamped.header.frame_id = tileType.ar_marker_id
        piece_CoM_pose_stamped.pose.position.x = tileType.centerOfMassOffset[0]
        piece_CoM_pose_stamped.pose.position.y = tileType.centerOfMassOffset[1]
        piece_CoM_pose_stamped.pose.position.z = 0
        piece_CoM_pose_stamped.pose.orientation.x = 0
        piece_CoM_pose_stamped.pose.orientation.y = 0
        piece_CoM_pose_stamped.pose.orientation.z = 0
        piece_CoM_pose_stamped.pose.orientation.w = 1
        
        transform_from_ar_marker_to_base = self.getTransformFromARTag(tileType.ar_marker_id)
        piece_CoM_pose_wrt_base = tf2_geometry_msgs.do_transform_pose(piece_CoM_pose_stamped, transform_from_ar_marker_to_base).pose
        print(piece_CoM_pose_wrt_base)

        x, y = piece_CoM_pose_wrt_base.position.x, piece_CoM_pose_wrt_base.position.y
        o_x, o_y, o_z, o_w = piece_CoM_pose_wrt_base.orientation.x, piece_CoM_pose_wrt_base.orientation.y, piece_CoM_pose_wrt_base.orientation.z, piece_CoM_pose_wrt_base.orientation.w
        e_x, e_y, e_z = euler_from_quaternion([o_x, o_y, o_z, o_w])

        # orientation may be off about x, y axis of AR tag
        # but we only care about the rotation about z axis (this is for figuring out the orientation of the gripper to pick up the piece in)
        new_ox, new_oy, new_oz, new_ow = quaternion_from_euler(0, 0, e_z - (np.pi / 2)) #off by 90 deg CW for some reason
        print(new_ox, new_oy, new_oz, new_ow)
        return self.move_to_pose_on_table(x, y, o_x=new_ow, o_y=new_oz, o_z=0.0, o_w=0.0)

    def move_to_piece_solution_pose(self, piece):
        piece_CoM_pose_wrt_base = self.getBasePoseForPiece(piece)
        return self.move_to_pose_on_board(x, y, o_x=new_ow, o_y=new_oz, o_z=0.0, o_w=0.0)        

    def getTransformFromBoard(self):
        return self.getTransformFromARTag(self.solver.board_ar_marker_id)

    def getBasePoseForPiece(self, piece):
        """
            get coordinates in board frame (wrt top left corner, i.e. ar_marker_8)
            transform those coordinates to base frame
        """
        #Pose of CoM w.r.t. top left AR tag of board
        board_pose_stamped = self.solver.getPoseForPiece(piece)
        transform_from_board_to_base = self.getTransformFromBoard()
        base_pose = tf2_geometry_msgs.do_transform_pose(board_pose_stamped, transform_from_board_to_base).pose
        return base_pose
