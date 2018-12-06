#!/usr/bin/env python
"""
Pick and Place script for Tetris Final Project
Author: James Fang
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

from path_planner import *
from pick_and_place import *
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper

z_offset = -.030
table_height = -0.195
def main():
    """
    left hand:
        [0.675, 0.007, 0.353]
    """
    pap = PickAndPlace(z_offset=z_offset)  
    addTableAtHeight(pap, table_height) 

    piece = pap.solver.getOrderForPlacement()[0]
    print("Piece: ", pap.solver.tileTypes[piece.tile_index].__name__)
    print("\tR: {}, C: {}, rot: {}".format(piece.row, piece.col, piece.rotation))
    print("\tCoordinates: {}".format(pap.solver.getCoordinatesForARTagOfPiece(piece)))
    print(pap.move_to_piece_CoM(piece))
    #pap.move_to_rotation(0.610, 0.032, table_height + 0.008, pap.ROT_0)
    #testMoveToRotation(pap, 0.763, -0.135, -0.170)
    #testRotateBy(pap)
    #resetGripper()
    #testGrasp(pap)
    #testRotations(pap)
    #testPickAndPlace(pap)

def testMoveToARMarker(pap):
    pap.table_height = table_height
    pap.move_to_ar_marker("ar_marker_0")

def addTableAtHeight(pap, z):
    pap.add_obstacle("table", 0.768, 0.032, z, 1.0, 1.0, 0.1)

def removeTable(pap):
    pap.remove_obstacle("table")

def resetGripper():
    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')
    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    #Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    #Open the right gripper
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)
    print('Done!')

def testMoveToRotation(pap, x, y, z):
    #GOES TO this random spot
    raw_input('Press [ Enter ]: ')
    pap.move_to_rotation(x, y, z, pap.ROT_0)
    rospy.sleep(1.0)

def testPickAndPlace(pap):
    startX, startY, startZ = 0.810, -0.114, -0.172
    endX, endY = 0.801, -0.315

    raw_input('Press [Enter] to pick up object at ({}, {}, {}): '.format(startX, startY, startZ))
    pap.move_to_rotation_and_grasp(startX, startY, startZ, pap.ROT_0)

    print("Table-height: ", pap.table_height)
    pap.move_to_rotation(startX, startY, pap.table_height + 0.05, pap.ROT_0)
    pap.move_to_rotation(endX, endY, pap.table_height + 0.05, pap.ROT_0)

    board_thickness = 0.00635 #cm = .25 inches
    raw_input('Press [Enter] to drop off object at ({}, {}, {}): '.format(endX, endY, pap.table_height))
    pap.move_to_rotation_and_open(endX, endY, pap.table_height + board_thickness, pap.ROT_0)

def testGrasp(pap):
    startX, startY, startZ = 0.691, 0.023, 0.069
    endX, endY, endZ = 0.810, -0.114, -0.170

    #GOES TO this random spot
    raw_input('Press [ Enter ]: ')
    pap.move_to_position(startX, startY, startZ)
    rospy.sleep(1.0)

    #Then goes to endx, endy, endz and tries to grasp an object there
    o_x, o_y, o_z, o_w = pap.rotations[2]
    raw_input('Press [ Enter ] for position 2: ')
    pap.move_to_pose_and_grasp(endX, endY, endZ, o_x=o_x, o_y=o_y, o_z=o_z, o_w=o_w)
    rospy.sleep(1.0)
    pap.openGripper()
    print("Table height: {}".format(pap.table_height))

def testRotations(pap):
    x, y, z = 0.828, -0.257, -0.024

    raw_input("Press <Enter> to move the right arm to goal pose 1: ")
    pap.move_to_rotation(x, y, z, pap.ROT_0)
    #rospy.sleep(1.0)
    #print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))

    raw_input("Press <Enter> to move the right arm to goal pose 2: ")   
    pap.move_to_rotation(x, y, z, pap.ROT_90)
    
    raw_input("Press <Enter> to move the right arm to goal pose 3: ")   
    pap.move_to_rotation(x, y, z, pap.ROT_180)
    
    raw_input("Press <Enter> to move the right arm to goal pose 4: ")   
    pap.move_to_rotation(x, y, z, pap.ROT_270)

def testRotateBy(pap):
    x, y, z = 0.815, -0.139, -0.151

    raw_input("Press <Enter> to move the right arm to goal pose 1: ")
    pap.move_to_rotation(x, y, z, pap.ROT_0)
    #rospy.sleep(1.0)
    #print(pap.isInDesiredRotation([o_x, o_y, o_z, o_w]))

    while not rospy.is_shutdown():
        raw_input("Press <Enter> to rotate by 90 deg: ")   
        pap.rotateBy(np.pi / 2)

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()




































    """
    ## Add the obstacle to the planning scene here
    #LENGTHS
    X = 1.20
    Y = 1.10
    Z = 0.1
    box_size = np.array([X, Y, Z])
    box_name = "table"

    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    #x, y, and z position 0.787, -0.475, -0.172
    box_pose.pose.position.x = 0.787
    box_pose.pose.position.y = -0.475
    box_pose.pose.position.z = -0.174 - Z
    #Orientation as a quaternion
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0

    #planner.add_box_obstacle(box_size, box_name, box_pose)
    #planner.remove_obstacle(box_name)
    
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
    """