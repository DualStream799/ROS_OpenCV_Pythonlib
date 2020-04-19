#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = "DualStream799"

class ControlBotModule():
    """docstring for TurtleBot"""

    def __init__(self):
        # Importing ROS related Libraries:
        from sensor_msgs.msg import Image, CompressedImage  # for '/kamera' Subscriber
        from geometry_msgs.msg import Twist, Vector3        # for '/cmd_vel' Publisher
        from sensor_msgs.msg import LaserScan               # for '/scan'   Subscriber
        from nav_msgs.msg import Odometry                   # for '/odom'   Subscriber
        from std_msgs.msg import UInt8                      # for '/bumper' Subscriber
        import rospy
        # Importing other needed libraries:
        from cv_bridge import CvBridge, CvBridgeError
        import numpy as np
        # Setting imported libaries available outside '__init__' scope but inside 'ControlBotModule' scope (Imports only when the class is instanced and let them available to all class' methods):
        self.Image, self.CompressedImage = Image, CompressedImage
        self.Twist, self.Vector3 = Twist, Vector3
        self.LaserScan = LaserScan
        self.Odometry = Odometry
        self.UInt8 = UInt8
        self.rospy = rospy
        self.CvBridge, self.CvBridgeError = CvBridge, CvBridgeError
        self.np = np
        # Instance needed to convert the images from "/kamera" to OpenCV especifications:
        self.cv_bridge = self.CvBridge()

        # Recieves the "/scan" Subscriber data:
        self.scan_data = []
        # Recieves the "/bumper" Subscriber data:
        self.bumper = 0
        # Recieves the "scan_data" element corresponding with frontal distance:
        self.ahead_fisrt = 0
        self.ahead_last = 0
        # Standard speed for insert on 'Vector3' objects:
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0
        self.angular_x = 0
        self.angular_y = 0
        self.angular_z = 0

    
    # METHODS FOR DEALING WITH ROS DATA SENSORS:
    def laser_scan(self, data):
        """Deals with '/scan' Subscriber data"""
        self.scan_data = self.np.array(data.ranges).round(decimals=2)
        self.ahead_fisrt = self.scan_data[0]
        self.ahead_last = self.scan_data[-1]


    def bumper_scan(self, dado):
        """Deals with '/bumper' Subscriber data"""
        self.bumper = dado.data


    def convert_compressed_to_cv2(self, image):
        """Deals with '/kamera' Subscriber data"""
        return self.cv_bridge.compressed_imgmsg_to_cv2(image, "bgr8")


    # METHODS FOR ROS MOVIMENTATION:
    def main_twist(self):
        """Returns a Twist object with all the linear and algular values on it"""
        return self.Twist(self.Vector3(self.linear_x, self.linear_y, self.linear_z), self.Vector3(self.angular_x, self.angular_y, self.angular_z))


class VisionBotModule():
    """docstring for VisionBotModule"""

    def __init__(self):
        # Importing OpenCV related Libraries:
        import numpy as np
        import cv2
        # Setting imported libaries available outside '__init__' scope but inside 'ControlBotModule' scope (Imports only when the class is instanced and let them available to all class' methods):
        self.cv2 = cv2
        self.np = np


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


    def frame_mask_hsv(self, hsv_frame, hue_value, hue_range_width, saturation_range=[0, 255], value_range=[0, 255]):
        """Creates a mask based on a given Hue value from the standard HSV colorspace and does the convertions to OpenCV HSV colorspace (standard HSV range: 0-360 / OpenCV HSV range: 0-180).
        By default Saturation and Value components doesn't have filters"""
        color_high = self.np.array((hue_value/2 + hue_range_width, saturation_range[1], value_range[1]))
        color_low = self.np.array((hue_value/2 - hue_range_width, saturation_range[0], value_range[0]))
        return self.cv2.inRange(hsv_frame, color_low, color_high)


    def frame_capture(self, filename, frame):
        """Saves a frame on a .jpg file"""
        self.cv2.imwrite(filename + '.jpg', frame)


    def contour_detection(self, canny_frame, retrieval_mode=None, approx_method=None):
        """Delimitates contours on a canny filtered frame"""
        if retrieval_mode is None:
            retrieval_mode = self.cv2.RETR_TREE
        if approx_method is None:
            approx_method = self.cv2.CHAIN_APPROX_SIMPLE
        contours, tree = self.cv2.findContours(canny_frame, retrieval_mode, approx_method)
        return contours
    

    def display_contours(self, frame, contours_list, mode=-1, color=[0, 255, 0], thickness=2):
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


    # METHODS FOR DISPLAY CONFIGURATIONS:
    def display_aim(self, rgb_frame, point, color=[0, 255, 255], width=2, length=4):
        """Draws a '+' over a given point"""
        self.cv2.line(rgb_frame, (point[0] - length/2, point[1]), (point[0] + length/2, point[1]), color, width, length)
        self.cv2.line(rgb_frame, (point[0], point[1] - length/2), (point[0], point[1] + length/2), color, width, length) 
    

    def display_text(self, frame, text, position, thickness, font_size=1, text_color=(255, 255, 255), shadow_color=(128, 128, 128), font_style=None, line_style=None):
        """Displays a text on the frame with a shadow behind it for better visualization on any background"""
        if font_style is None:
            font_style = self.cv2.FONT_HERSHEY_SIMPLEX
        if line_style is None:
            line_style = self.cv2.LINE_AA
        self.cv2.putText(frame, text, position, font_style, font_size, shadow_color, thickness+1, line_style)
        self.cv2.putText(frame, text, position, font_style, font_size, text_color, thickness, line_style)


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


    def convert_lines_angular_to_linear(self, rho, theta):
        """Calculates two points given a 'rho' and 'theta' values of a line (usefull to deals with the results of 'cv2.HoughLines')"""
        a = self.np.cos(theta)
        b = self.np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        return (int(x0 + 1000*(-b)), int(y0 + 1000*(a))), (int(x0 - 1000*(-b)) , int(y0 - 1000*(a)))


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
