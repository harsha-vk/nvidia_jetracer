#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import message_filters
import numpy as np
import cv2

class LaneDetection(Node):

	def __init__(self):
        super().__init__('lane_detection')

        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length'),
                               				('camera'),
                               				('canny/k_size', 1, addDescription(1, 16, 2)),
                               				('canny/min_threshold', 0, addDescription(0, 256, 1)),
                                            ('canny/max_threshold', 255, addDescription(0, 256, 1)),
                                            ('hls/max_threshold', 0, addDescription(0, 256, 1)),
                                            ('hls/min_threshold', 255, addDescription(0, 256, 1)),
                                            ('top_mid_point'),
                                            ('bottom_mid_point')])

        self.add_on_set_parameters_callback(self.paramsCallback)

        self.nodeParams()

        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # Create Subscribers
        self.left_sub = message_filters.Subscriber(self, Image, 'left/image_rect', qos_profile=qos_profile)
        self.right_sub = message_filters.Subscriber(self, Image, 'right/image_rect', qos_profile=qos_profile)

        # Apply message filter
        self.timestamp_sync = message_filters.TimeSynchronizer([self.left_sub, self.right_sub], queue_size=10)
        self.timestamp_sync.registerCallback(self.imageCallback)

        # Load cv_bridge
        self.bridge = CvBridge()

    def addDescription(self, from_value=None, to_value=None, step=None):
        integer_range = None
        descriptor = None
        integer_range = IntegerRange()
        integer_range.from_value = from_value
        integer_range.to_value = to_value
        descriptor = ParameterDescriptor()
        descriptor.description = "-"
        descriptor.integer_range = [integer_range]
        return descriptor

    def paramsCallback(self, params):
        success = False
    	for param in params:
	        if param.name == 'camera' and param.type_ == Parameter.Type.STRING:
                if param.value in ['left', 'right']:
                    self.camera = param.value
                    self.sliding_window = True
                    success = True
            if param.name == 'canny/k_size' and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(1, 16, 2):
                    self.K = param.value
                    success = True
            if param.name == 'canny/min_threshold' and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(256):
                    self.C_T[0] = param.value
                    success = True
            if param.name == 'canny/max_threshold' and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(256):
                    self.C_T[1] = param.value
                    success = True
            if param.name == 'hls/min_threshold' and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(256):
                    self.H_T[0] = param.value
                    success = True
            if param.name == 'hls/max_threshold' and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(256):
                    self.H_T[1] = param.value
                    success = True
            if (param.name in ['top_margin', 'bottom_margin']) and param.type_ == Parameter.Type.INTEGER:
                if param.value in range(0, self.width):
                    self.do_once = True
                    success = True
	    return SetParametersResult(successful=success)

    def nodeParams(self):
        self.camera = self.get_parameter('camera').get_parameter_value().string_value
        self.K = self.get_parameter('canny/k_size').get_parameter_value().integer_value
        self.C_T = [0, 0]
        self.C_T[0] = self.get_parameter('canny/min_threshold').get_parameter_value().integer_value
        self.C_T[1] = self.get_parameter('canny/max_threshold').get_parameter_value().integer_value
        self.H_T = [0, 0]
        self.H_T[0] = self.get_parameter('hls/min_threshold').get_parameter_value().integer_value
        self.H_T[1] = self.get_parameter('hls/max_threshold').get_parameter_value().integer_value
        self.do_once = True
        self.declared = False
        self.sliding_window = True

    def imageCallback(self, left_msg, right_msg):
    	if self.camera == 'right':
    		img = self.bridge.imgmsg_to_cv2(right_msg)
            encoding = right_msg.encoding
        else:
        	img = self.bridge.imgmsg_to_cv2(left_msg)
            encoding = left_msg.encoding

        canny_binary = canny(img, encoding, self.K, self.C_T)
        
        if encoding == "bgr8":
            hls_binary = hlsColorSpace(img, self.H_T)
            combined_binary = np.zeros_like(canny_binary)
            combined_binary[(canny_binary == 1) | (hls_binary == 1)] = 1
            img = combined_binary
        else:
            img = canny_binary

        if self.do_once == True:
            self.height = img.shape[0]
            self.width = img.shape[1]

            if self.declared == False:
                self.declare_parameters(namespace='', parameters=[('top_margin', 0, addDescription(0, self.width, 1)),
                                                                  ('bottom_margin', 0, addDescription(0, self.width, 1))])
                self.declared = True

            t_p = self.get_parameter('top_mid_point').get_parameter_value().integer_array_value
            b_p = self.get_parameter('bottom_mid_point').get_parameter_value().integer_array_value
            t_m = self.get_parameter('top_margin').get_parameter_value().integer_value
            b_m = self.get_parameter('bottom_margin').get_parameter_value().integer_value
            src = np.float32([[(t_p[0] + t_m), t_p[1]],
                              [(b_p[0] + b_m), b_p[1]],
                              [(b_p[0] - b_m), b_p[1]],
                              [(t_p[0] - t_m), t_p[1]]])
            dst = np.float32([[self.width, 0], [self.width, self.height], [0, self.height], [0, 0]])
            self.M = cv2.getPerspectiveTransform(src, dst)
            self.Minv = cv2.getPerspectiveTransform(dst, src)
            self.do_once = False

        # Perspective Transform
        img = cv2.warpPerspective(img, self.M, img.shape[::-1], flags=cv2.INTER_LINEAR)

        if self.sliding_window == True:
            findLanePixels(img)
        else:
            searchAroundPoly(img)

    def canny(img, encoding, k=1, t=[0, 255]):
    	if encoding == "bgr8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	    img = cv2.GaussianBlur(img, (k, k), 0)
	    img = cv2.Canny(img, t[0], t[1])
	    img = img / 255
	    return img

    def hlsColorSpace(img, t=[0, 255]):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        img = hls[:,:,2]
        out = np.zeros_like(img)
        out[(img > t[0]) & (img <= t[1])] = 1
        return out

    def findLanePixels(img):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 10
        margin = 25
        minpix = 20
        window_height = np.int64(binary_warped.shape[0]//nwindows)

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > minpix:
                leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int64(np.mean(nonzerox[good_right_inds]))

        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        return leftx, lefty, rightx, righty

    def searchAroundPoly(binary_warped, left_fit, right_fit):
        margin = 20

        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + 
                        left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + 
                        left_fit[1]*nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + 
                        right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + 
                        right_fit[1]*nonzeroy + right_fit[2] + margin)))

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        return leftx, lefty, rightx, righty