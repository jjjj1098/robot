#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

class ImageSub():
	def __init__(self):
	# subscribes raw image
		self.sub_image_original = rospy.Subscriber('/camera/image', Image, self.cvImage, queue_size = 1)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.cvBridge = CvBridge()
		self.twist = Twist()

#		parking_state = False
		stop_state = False

#		self.parking_state(parking_state)
		self.stop_state(stop_state)

#		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#		self.twist = Twist()

#	def parking_state(self, state):
#		global parking_detect
#		parking_detect = state

	def stop_state (self, state):
		global stop_detect
		stop_detect = state

	def cvImage(self, msg_img):
		cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")
		color_img = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2HSV)

		rows, cols, channels = cv_image_original.shape

		y_left = self.line_detect(cv_image_original, np.array([10, 10, 10]), np.array([255,255,250]), 40, rows, cols, color_img)
		w_right = self.line_detect(cv_image_original, np.array([0,0,250]), np.array([0,0,255]), 280, rows, cols, color_img)

		cv2.imshow("Image window", cv_image_original)
		cv2.waitKey(3)

		
#		if parking_detect == False:
#			parking = self.signal_detect (cv_image_original, np.array([90,50,70]), np.array([128,255,255]), rows, cols, color_img)
#		else:
#			parking = 0

#		if parking > 3600 and parking_detect == False:
#			self.parking_state(True)
#			self.moving(y_left, w_right, cols, 0.0, 0.0)
#			print()
			
#		cv2.imshow("Image window", cv_image_original)
#		cv2.waitKey(3)

		if stop_detect == False:
			stop = self.signal_detect (cv_image_original, np.array([159,50,70]), np.array([180,255,255]), rows, cols, color_img)
		else:
			stop = 0

		if stop > 1300 and stop_detect == False:
			self.stop_state(True)
			self.moving (y_left, w_right, cols, 0.0, 0.0)
			print()

		if stop_detect == True:
			self.moving(y_left, w_right, cols, 0.0, 0.0)
		else:
			self.moving(y_left, w_right, cols, 0.1, 1)

		cv2.imshow("Image window", cv_image_original)
		cv2.waitKey(3)


	def line_detect (self, cv_image_original, y_left, w_right, ideal_p, rows, cols, color_img):


		mask = cv2.inRange(color_img, y_left, w_right)

		x = ideal_p
		y = 190

		top = 3*rows/4
		bot = 3*rows/4 + 20

		mask[0:int(top), 0:cols] = 0
		mask[int(bot):rows, 0:cols] = 0

		M = cv2.moments(mask)
		if M['m00'] > 0:
			x = int(M['m10']/M['m00'])
			y = int(M['m01']/M['m00'])
			cv2.circle(cv_image_original, (x, y), 20, (0,0,255), -1)
		else:
			cv2.circle(cv_image_original, (x, y), 20, (0,255,0), -1)

		return x
		
	def signal_detect (self, cv_image_original, lower, upper, rows, cols, color_img):

		mask = cv2.inRange(color_img, lower, upper)

		mask[0:int(rows/2), 0:int(rows/2)] = 0
		mask[int(rows/2):rows, 0:cols] = 0

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
		opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

		contours = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = contours[0] if len (contours) == 2 else contours[1]

		area = 0

		for pixel in contours:
			area += cv2.contourArea(pixel)

		return area

	def moving (self, y_left, w_right, cols, vel, ang):

		err = ((y_left+w_right)/2) - (cols/2)
		self.twist.linear.x = vel
		self.twist.angular.z = -float(err) / 100 * ang
		self.cmd_vel_pub.publish(self.twist)


rospy.init_node('image_sub')
image_sub = ImageSub()
rospy.spin()
