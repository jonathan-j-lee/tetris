#!/usr/bin/env python

from __future__ import division, generators, print_function, unicode_literals

import matplotlib.pyplot as plt
import numpy as np
import rospy
from scipy import signal
from sensor_msgs.msg import Image

LEFT_CAM = '/cameras/left_hand_camera/image'
RIGHT_CAM = '/cameras/right_hand_camera/image'

GAUSSIAN_KERNEL = np.outer(signal.gaussian(10, 2), signal.gaussian(10, 2))

SUCTION_CAM = RIGHT_CAM  # FIXME: make a command-line option

SHOWN = False  # HACK

def rgb_to_grayscale(r, g, b):
	""" Approximate an RGB image as grayscale. """
	return 0.3*r + 0.59*g + 0.11*b

def suction_cam_callback(image):
	assert image.encoding == 'bgra8'
	raw = np.fromstring(image.data, dtype=np.uint8).reshape((image.height, image.width, 4))
	grayscale = rgb_to_grayscale(raw[:, :, 0], raw[:, :, 1], raw[:, :, 2]).astype(np.uint8)
	blurred = signal.fftconvolve(grayscale, GAUSSIAN_KERNEL, mode='same')
	# blurred = grayscale
	global SHOWN
	if not SHOWN:
		SHOWN = True
		plt.imshow(blurred, cmap='gray')
		plt.show()

def main():
	rospy.init_node('vision')
	rospy.Subscriber(SUCTION_CAM, Image, suction_cam_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
