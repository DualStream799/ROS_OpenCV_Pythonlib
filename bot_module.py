#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = "DualStream799"


class SupportBotModule():
	"""docstring for SupportBotModule"""
	
	def __init__(self):
		# Importing Support related Libraries:
		from heapq import nlargest, nsmallest
		import numpy as np
		import math
		# Setting imported libaries available outside '__init__' scope but inside 'ControlBotModule' scope (Imports only when the class is instanced and let them available to all class' methods):
		self.nlargest, self.nsmallest = nlargest, nsmallest
		self.np = np
		self.math = math


	def calculate_hypotenuse_2D(self, point):
		"""Calculates hypotenuse given x and y values of a point (x, y)"""
		return self.math.sqrt(point[0]**2 + point[1]**2)


	def calculate_projection(self, point1, point2, module=True):
		"""Calculates line's projection over an axis (function return the module of the subtraction, for negative values set 'module' to False)
		x-axis projection = 0 : vertical lines (0º)
		y-axis projection = 0 : horizontal lines (180º)
		diff of both projections = 0 : diagonal lines (45º)"""
		proj_x, proj_y = [point1[0] - point2[0], point1[1] - point2[1]]
		proj_diff = proj_x - proj_y
		if module:
			return abs(proj_x), abs(proj_y), abs(proj_diff)
		else:
			return proj_x, proj_y, proj_diff

	
	def angular_coefficient(self, point1, point2, decimals=0, to_degrees=False):
		"""Calculates the angular coefficient if a line between two points using the current formula: (y - y0) = m*(x - x0)"""
		m = (point2[1] - point1[1])/(point2[0] - point1[0])
		if to_degrees == False:
			return m
		else:
			return round(self.math.degrees(self.math.atan(m)), decimals)

	def calculate_vanishing_point(self, p1, p2, q1, q2):
		"""Calculates vanishing point based in the intersection of two lines"""
		def calculate_h(point, m):
			return point[1] - m*point[0]

		m1 = self.angular_coefficient(p1, p2)
		m2 = self.angular_coefficient(q1, q2)
		
		h1 = calculate_h(p1, m1)
		h2 = calculate_h(q1, m2)
		
		xi = (h2 - h1)/(m1 - m2)
		yi = m1*xi + h1
		
		return (int(xi), int(yi))


	def convert_lines_angular_to_linear(self, rho, theta):
		"""Calculates two points given a 'rho' and 'theta' values of a line (usefull to deals with the results of 'cv2.HoughLines')"""
		a = self.np.cos(theta)
		b = self.np.sin(theta)
		x0 = a*rho
		y0 = b*rho
		return (int(x0 + 1000*(-b)), int(y0 + 1000*(a))), (int(x0 - 1000*(-b)) , int(y0 - 1000*(a)))


	def convert_dimensions_to_points(self, dimensions, w_multiplier=1, h_multiplier=1):
		""" Convert dimensions (x, y, w, h) into two points (x0, y0) e (x1, y1) on any scale proportion"""
		return (int(dimensions[0]), int(dimensions[1])), (int(dimensions[0]+dimensions[2]*w_multiplier), int(dimensions[1]+dimensions[3]*h_multiplier))



class ControlBotModule(SupportBotModule):
	"""docstring for TurtleBot"""

	def __init__(self):
		# Importing ROS related Libraries:
		from sensor_msgs.msg import Image, CompressedImage  # for '/kamera' Subscriber
		from geometry_msgs.msg import Twist, Vector3, Point # for '/cmd_vel' Publisher
		from sensor_msgs.msg import LaserScan               # for '/scan'   Subscriber
		from nav_msgs.msg import Odometry                   # for '/odom'   Subscriber
		from std_msgs.msg import UInt8                      # for '/bumper' Subscriber
		import rospy
		# Importing other needed libraries:
		from cv_bridge import CvBridge, CvBridgeError
		import tf
		import numpy as np
		import math
		# Setting imported libaries available outside '__init__' scope but inside 'ControlBotModule' scope (Imports only when the class is instanced and let them available to all class' methods):
		self.Image, self.CompressedImage = Image, CompressedImage
		self.Twist, self.Vector3, self.Point = Twist, Vector3, Point
		self.LaserScan = LaserScan
		self.Odometry = Odometry
		self.UInt8 = UInt8
		self.rospy = rospy
		self.CvBridge, self.CvBridgeError = CvBridge, CvBridgeError
		self.tf = tf
		self.np = np
		self.math = math
		# Instance needed to convert the images from "/kamera" to OpenCV especifications:
		self.cv_bridge = self.CvBridge()

		# Recieves the '/scan' Subscriber data:
		self.scan_data = []
		# Recieves the '/bumper' Subscriber data:
		self.bumper = 0
		# Recieves the '/scan_data' element corresponding with frontal distance:
		self.ahead_fisrt = 0
		self.ahead_last = 0
		# Reciever the '/odom' Subscriber data:
		self.odom_x = 0
		self.odom_y = 0
		self.odom_z = 0
		self.odom_roll = 0
		self.odom_pitch = 0
		self.odom_yaw = 0
		# Standard speed for insert on 'Vector3' objects:
		self.linear_x = 0
		self.linear_y = 0
		self.linear_z = 0
		self.angular_x = 0
		self.angular_y = 0
		self.angular_z = 0
		# A Goal defines a point on space where the bot aims to go:
		self.goal_point = self.Point()
		self.goal_angle = 0
		self.goal_distance = 0
		self.goal_orientation = [0, 0, 0]
	

	# METHODS FOR DEALING WITH ROS DATA SENSORS:
	def laser_scan(self, data):
		"""Deals with '/scan' Subscriber data"""
		self.scan_data = self.np.array(data.ranges).round(decimals=2)
		self.ahead_fisrt = self.scan_data[0]
		self.ahead_last = self.scan_data[-1]


	def bumper_scan(self, dado):
		"""Deals with '/bumper' Subscriber data"""
		self.bumper = dado.data


	def odom_scan(self, ros_msg):
		"""Deals with '/odom' Subscriber data"""
		# Position:
		self.odom_x = ros_msg.pose.pose.position.x
		self.odom_y = ros_msg.pose.pose.position.y
		self.odom_z = ros_msg.pose.pose.position.z
		# Orientation:
		self.odom_roll, self.odom_pitch, self.odom_yaw = self.convert_quaternion_to_radians(ros_msg.pose.pose.orientation)


	# METHODS FOR SENSOR DATA CONVERSION:
	def convert_compressed_to_cv2(self, image):
		"""Deals with '/kamera' Subscriber data"""
		return self.cv_bridge.compressed_imgmsg_to_cv2(image, "bgr8")


	def convert_quaternion_to_radians(self, orientation):
		"""Converts the odometry orientation from quaternion (representation system used on ROS) to radians rotations  on three axis"""
		roll, pitch, yaw = self.tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		return roll, pitch, yaw


	# METHODS FOR ROS MOVIMENTATION:
	def main_twist(self):
		"""Returns a Twist object with all the linear and algular values on it"""
		return self.Twist(self.Vector3(self.linear_x, self.linear_y, self.linear_z), self.Vector3(self.angular_x, self.angular_y, self.angular_z))

	def stop_twist(self):
		"""Returns a Twist object with all the linear and angular values zeroed"""
		return self.Twist(self.Vector3(0, 0, 0), self.Vector3(0, 0, 0))

	def set_goal(self, x=0, y=0, z=0):
		"""Defines a Point as a goal to be used as reference to go to"""
		self.goal_point.x = x
		self.goal_point.y = y
		self.goal_point.z = z

	def update_goal_state(self):
		diff_x = self.goal_point.x - self.odom_x
		diff_y = self.goal_point.y - self.odom_y
		self.goal_distance = self.calculate_hypotenuse_2D((diff_x, diff_y))
		self.goal_angle = self.math.atan2(diff_x, diff_y)
		if self.goal_angle > self.math.pi:
			self.goal_angle -= self.math.pi


class VisionBotModule(SupportBotModule):
	"""docstring for VisionBotModule"""

	def __init__(self):
		# Importing OpenCV related Libraries:
		from heapq import nlargest
		import numpy as np
		import cv2
		# Setting imported libaries available outside '__init__' scope but inside 'ControlBotModule' scope (Imports only when the class is instanced and let them available to all class' methods):
		self.nlargest = nlargest
		self.cv2 = cv2
		self.np = np
		#self.sup = SupportBotModule()

	# METHODS FOR COMPUTER VISION PROCESSMENTS:
	def frame_flip(self, frame, flip_mode):
		"""Recieves a webcam frame and flips it or not, depending on 'flip' argument
		'flip_mode' = 0 : X-axis Flip (Vertical)
		'flip_mode' > 0 : Y-axis Flip (Horizontal)
		'flip_mode' < 0 : Both-axis Flip(Vertical and Horizontal)"""
		return self.cv2.flip(frame, flip_mode)


	def frame_spacecolors(self, frame):
		"""Recieves a frame and returns frames on different spacecolors (and flipped)"""
		# Original frame (OpenCV's default frames are BGR):
		bgr_frame = frame
		# Converts frame from BGR to Grayscale:
		gray_frame = self.cv2.cvtColor(bgr_frame, self.cv2.COLOR_BGR2GRAY)
		# Converts frame from BGR to RGB:
		rgb_frame = self.cv2.cvtColor(bgr_frame, self.cv2.COLOR_BGR2RGB)
		# Converts frame from RGB to HSV:
		hsv_frame = self.cv2.cvtColor(rgb_frame, self.cv2.COLOR_RGB2HSV)
		
		return bgr_frame, gray_frame, rgb_frame, hsv_frame


	def frame_mask_hsv(self, hsv_frame, hue_value, hue_range_width, saturation_range=(0, 255), value_range=(0, 255)):
		"""Creates a mask based on a given Hue value from the standard HSV colorspace and does the convertions to OpenCV HSV colorspace (standard HSV range: 0-360 / OpenCV HSV range: 0-180).
		By default Saturation and Value components doesn't have filters"""
		if type(hue_range_width) == int:
			hue_range_width = (hue_range_width, hue_range_width)
		color_high = self.np.array((hue_value/2 + hue_range_width[1], saturation_range[1], value_range[1]))
		color_low = self.np.array((hue_value/2 - hue_range_width[0], saturation_range[0], value_range[0]))
		return self.cv2.inRange(hsv_frame, color_low, color_high)


	def frame_capture(self, filename, frame):
		"""Saves a frame on a .jpg file"""
		self.cv2.imwrite(filename + '.jpg', frame)


	def contour_detection(self, canny_frame, retrieval_mode=None, approx_method=None):
		"""Delimitates contours on a canny filtered frame"""
		if retrieval_mode is None:
			retrieval_mode = self.cv2.RETR_TREE
		if approx_method is None:
			approx_method = self.cv2.CHAIN_APPROX_NONE
		elif approx_method is 'simple':
			approx_method = self.cv2.CHAIN_APPROX_SIMPLE
		contours, tree = self.cv2.findContours(canny_frame, retrieval_mode, approx_method)
		return contours, tree


	def contour_biggest_area(self, contours):
		"""Returns the contour which has the biggest area"""
		return self.nlargest(1, contours, key=lambda x: self.cv2.contourArea(x))[0]
		#return biggest_contour


	def contour_features(self, contour, mode):
		"""Calculates a contour's feature based on a valid 'mode' argumment.
		'mode' = 'area'      : returns float
		'mode' = 'center'    : returns tuple  (Cx, Cy)
		'mode' = 'perimeter' : returns float
		'mode' = 'str-rect'  : returns tuple  (x, y, w, h)
		'mode' = 'min-rect'  : returns list   [4 corners]
		'mode' = 'min-circle': returns list   [(x,y), radius]
		"""
		# Calculates the area:
		if mode == 'area':
			return self.cv2.contourArea(contour)
		# Calculates the centroid:
		elif mode == 'center':
			m = self.cv2.moments(contour)
			return ( int(m['m10']/m['m00']), int(m['m01']/m['m00']) )
		# Calculates the perimeter:
		elif mode == 'perimeter':
			return self.cv2.arcLenght(contour)
		# Calculates the straight rectangle around a contour (doesn't consider rotation):
		elif mode == 'str-rect':
			return self.cv2.boundingRect(contour)
		# Calculates the minimum area rectangle around a contour (considers rotation):
		elif mode == 'min-rect':
			return [self.np.int0(self.cv2.boxPoints(self.cv2.minAreaRect(contour)))]
		# Calculates the minimum area circle around a contour:
		elif mode == 'min-circle':
			(x,y), radius = self.cv2.minEnclosingCircle(contour)
			return [(int(x), int(y)), int(radius)]


	def contour_draw(self, frame, contours_list, mode=-1, color=[0, 255, 0], thickness=2):
		"""Draws all contours on a frame in a given color and border thickness
		'mode' = -1 : contours' borders
		'mode' = 0  : contours filled"""
		self.cv2.drawContours(frame, contours_list, mode, color, thickness)


	def morphological_transformation(self, frame, mode, kernel_size):
		"""Realizes different morphological transformations"""
		# Creates kernel:
		kernel = self.np.ones((kernel_size, kernel_size), self.np.uint8)
		# Transformation selector:
		if mode == 'erosion':
			return self.cv2.erode(frame, kernel, iterations=1)
		elif mode == 'dilation':
			return self.cv2.dilate(frame, kernel, iterations=1)
		elif mode == 'opening':
			return self.cv2.morphologyEx(frame, self.cv2.MORPH_OPEN, kernel)
		elif mode == 'closing':
			return self.cv2.morphologyEx(frame, self.cv2.MORPH_CLOSE, kernel)
		elif mode == 'gradient':
			return self.cv2.morphologyEx(frame, self.cv2.MORPH_GRADIENT, kernel)
		elif mode == 'tophat':
			return self.cv2.morphologyEx(frame, self.cv2.MORPH_TOPHAT, kernel)
		elif mode == 'blackhat':
			return self.cv2.morphologyEx(frame, self.cv2.MORPH_BLACKHAT, kernel)


	# METHODS FOR DISPLAY ELEMENTS:
	def display_frame(self, title, frame):
		"""Displays a frame"""
		self.cv2.imshow(title, frame)

	def display_terminate(self, title=None):
		if title is not None:
			self.cv2.destroyWindow(title)
		else:
			self.cv2.destroyAllWindows()


	def draw_text(self, frame, text, position, thickness=2, font_size=1, text_color=(255, 255, 255), shadow_color=(128, 128, 128), font_style=None, line_style=None):
		"""Draws a text on the frame with a shadow behind it for better visualization on any background"""
		if font_style is None:
			font_style = self.cv2.FONT_HERSHEY_SIMPLEX
		if line_style is None:
			line_style = self.cv2.LINE_AA
		self.cv2.putText(frame, text, position, font_style, font_size, shadow_color, thickness+1, line_style)
		self.cv2.putText(frame, text, position, font_style, font_size, text_color, thickness, line_style)


	def draw_aim(self, rgb_frame, point, color=(0, 255, 255), width=2, length=8):
		"""Draws a aim ('+' symbol) over a given point on the frame"""
		self.cv2.line(rgb_frame, (point[0] - length/2, point[1]), (point[0] + length/2, point[1]), color, width, length)
		self.cv2.line(rgb_frame, (point[0], point[1] - length/2), (point[0], point[1] + length/2), color, width, length) 
	

	def draw_rectangle(self, frame, dimensions, color=(0, 255, 255), scale=(1,1)):
		"""Draws a rectangle on the frame dealing directly with cv2.boundingRect (present in 'contour_features' method if mode='str-rect') returned data
		('dimensions' parameters expects a tuple of 4 values (x, y, w, h) returned from cv2.boundingRect"""
		p1, p2 = self.convert_dimensions_to_points(dimensions, scale[0], scale[1])
		self.cv2.rectangle(frame, p1, p2, color, thickness=2)


