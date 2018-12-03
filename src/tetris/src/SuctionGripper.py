#!/usr/bin/env python
"""
Suction gripping script for Lab 8
Author: James Fang
"""
import rospy
import baxter_interface
from baxter_interface import Gripper

class SuctionGripper(object):

	def __init__(self, limb='right'):
		self.gripper = Gripper(limb)
		print("---------{} gripper type: {}".format(limb, self.gripper.type()))
		self.gripper_vacuum_sensor = baxter_interface.AnalogIO('{}_vacuum_sensor_analog'.format(limb))

	def close(self):
		#Close the gripper
		print('Closing...')
		self.gripper.close()
		self.is_grasping()
		rospy.sleep(3.0)
		self.is_grasping()

	def open(self):
		#Open the right gripper
		print('Opening...')
		self.gripper.open()
		self.is_grasping()
		rospy.sleep(1.0)
		self.is_grasping()
		print('Done!')

	def is_grasping(self):
		"""
		Both the topics /robot/analog_io/left_vacuum_sensor_analog/value_uint32 and /robot/analog_io/left_vacuum_sensor_analog/state 
		give you the direct analog readings from the vacuum sensor in the vacuum gripper. According to our gripper engineer, the values mean:

		Value    	Meaning
		0-46 :    	The vacuum gripper is likely not attached to an object
		47-175: 	The vacuum is likely attached to an object (usually around 150 when grasping)
		175+:    	There is likely a short between 5V and signal on the sensor.
		"""
		print("-------------Curr vacuum state:", self.gripper_vacuum_sensor.state())
		return self.gripper_vacuum_sensor.state() > 100