#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nvidia_jetracer.msg import LaneGeometry

class LaneDetection(Node):

	def __init__(self):
        super().__init__('lane_detection')

        # Declare ROS parameters
        self.declare_parameters(namespace='',
                               	parameters=[('qos_length'),
                               				('canny/k_size',1,self.addDescription(1,15,2)),
                               				('canny/min_threshold',0,self.addDescription(0,255,1)),
                                            ('canny/max_threshold',255,self.addDescription(0,255,1)),
                                            ('hls/max_threshold',0,self.addDescription(0,255,1)),
                                            ('hls/min_threshold',255,self.addDescription(0,255,1)),
                                            ('sliding_window/num_windows',5,self.addDescription(5,30,1)),
                                            ('sliding_window/margin',5,self.addDescription(5,100,1)),
                                            ('sliding_window/min_pixels',5,self.addDescription(5,50,1)),
                                            ('lane_line/margin',5,self.addDescription(5,100,1))])

        self.set_parameters_callback(self.paramsCallback)

        self.nodeParams()

        qos_length = self.get_parameter('qos_length').get_parameter_value().integer_value
        qos_profile = QoSProfile(depth=qos_length,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 reliability=QoSReliabilityPolicy.RELIABLE)


        # Load cv_bridge
        self.bridge = CvBridge()

        # Create Subscribers
        self.left_img_sub = self.create_subscription(Image,'left/image_rect',self.imageCallback,qos_profile)

        # Create Publishers
        self.lane_geo_pub = self.create_publisher(LaneGeometry,'lane_geometry',qos_profile)

    def addDescription(self,from_value=None,to_value=None,step=None):
        descriptor = ParameterDescriptor()
        if None not in [from_value,to_value,step]:
            integer_range = IntegerRange()
            integer_range.from_value = from_value
            integer_range.to_value = to_value
            descriptor.integer_range = [integer_range]
        descriptor.description = "-"
        return descriptor

    def paramsCallback(self,params):
        success = False
    	for param in params:
            if param.type_ == Parameter.Type.INTEGER:
                if param.name == 'canny/k_size':
                    if param.value in range(1,16,2):
                        self.K = param.value
                        success = True
                elif param.name == 'canny/min_threshold':
                    if param.value in range(256):
                        self.C_T[0] = param.value
                        success = True
                elif param.name == 'canny/max_threshold':
                    if param.value in range(256):
                        self.C_T[1] = param.value
                        success = True
                elif param.name == 'hls/min_threshold':
                    if param.value in range(256):
                        self.H_T[0] = param.value
                        success = True
                elif param.name == 'hls/max_threshold':
                    if param.value in range(256):
                        self.H_T[1] = param.value
                        success = True
                elif param.name == 'sliding_window/num':
                    if param.value in range(5,31,1):
                        self.f_nwindows = param.value
                        success = True
                elif param.name == 'sliding_window/margin':
                    if param.value in range(5,101,1):
                        self.f_margin = param.value
                        success = True
                elif param.name == 'sliding_window/min_pixels':
                    if param.value in range(5,51,1):
                        self.f_minpix = param.value
                        success = True
                elif param.name == 'lane_line/margin':
                    if param.value in range(5,101,1):
                        self.s_margin = param.value
                        success = True
                elif param.name in ['roi/top/left_offset','roi/top/right_offset',
                                    'roi/bottom/left_offset','roi/bottom/right_offset',
                                    'roi/top_offset','roi/bottom_offset']:
                    self.do_on_set = True
                    success = True
	    return SetParametersResult(successful=success)

    def nodeParams(self):
        self.K = self.get_parameter('canny/k_size').get_parameter_value().integer_value
        self.C_T = [0,0]
        self.C_T[0] = self.get_parameter('canny/min_threshold').get_parameter_value().integer_value
        self.C_T[1] = self.get_parameter('canny/max_threshold').get_parameter_value().integer_value
        self.H_T = [0,0]
        self.H_T[0] = self.get_parameter('hls/min_threshold').get_parameter_value().integer_value
        self.H_T[1] = self.get_parameter('hls/max_threshold').get_parameter_value().integer_value
        self.do_on_set = True
        self.declared = False
        self.sliding_window = True
        self.f_nwindows = self.get_parameter('sliding_window/num_windows').get_parameter_value().integer_value
        self.f_margin = self.get_parameter('sliding_window/margin').get_parameter_value().integer_value
        self.f_minpix = self.get_parameter('sliding_window/min_pixels').get_parameter_value().integer_value
        self.s_margin = self.get_parameter('lane_line/margin').get_parameter_value().integer_value

    def imageCallback(self,left_msg):
    	img = self.bridge.imgmsg_to_cv2(left_msg)
        encoding = left_msg.encoding

        canny_binary = self.canny(img,encoding,self.K,self.C_T)
        
        if encoding == "bgr8":
            hls_binary = self.hlsColorSpace(img,self.H_T)
            combined_binary = np.zeros_like(canny_binary)
            combined_binary[(canny_binary == 1) | (hls_binary == 1)] = 1
            img = combined_binary
        else:
            img = canny_binary

        img = self.perspectiveTransform(img)

        if self.sliding_window == True:
            self.findLanePixels(img)
        else:
            self.searchAroundPoly(img)

        lanemsg = LaneGeometry()
        lanemsg.header.stamp = left_msg.header.stamp
        lanemsg.header.frame_id = 'lane_geometry_frame'
        lanemsg.left_params = self.left_lane_fit.tolist()
        lanemsg.right_params = self.right_lane_fit.tolist()
        self.lane_geo_pub.publish(lanemsg)

    def canny(self,img,encoding,k=1,t=[0,255]):
    	if encoding == "bgr8":
            img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	    img = cv2.GaussianBlur(img,(k,k),0)
	    img = cv2.Canny(img,t[0],t[1])
	    img = img // 255
	    return img

    def hlsColorSpace(self,img,t=[0,255]):
        img = cv2.cvtColor(img,cv2.COLOR_BGR2HLS)
        img = hls[:,:,2]
        out = np.zeros_like(img)
        out[(img > t[0]) & (img <= t[1])] = 1
        return out

    def perspectiveTransform(self,img):
        if self.do_on_set == True:
            self.height = img.shape[0]
            self.width = img.shape[1]

            if self.declared == False:
                self.declare_parameters(namespace='',
                                        parameters=[('roi/top/left_offset',0,addDescription(0,self.width//2,1)),
                                                    ('roi/top/right_offset',self.width//2,addDescription(self.width//2,self.width,1)),
                                                    ('roi/bottom/left_offset',0,addDescription(0,self.width//2,1)),
                                                    ('roi/bottom/right_offset',self.width//2,addDescription(self.width//2,self.width,1)),
                                                    ('roi/top_offset',0,addDescription(0,3*self.height//4,1)),
                                                    ('roi/bottom_offset',self.height,addDescription(self.height,3*self.height//4,-1))])
                self.declared = True

            txlo = self.get_parameter('roi/top/left_offset').get_parameter_value().integer_value
            txro = self.get_parameter('roi/top/right_offset').get_parameter_value().integer_value
            bxlo = self.get_parameter('roi/bottom/left_offset').get_parameter_value().integer_value
            bxro = self.get_parameter('roi/bottom/right_offset').get_parameter_value().integer_value
            tyo = self.get_parameter('roi/top_offset').get_parameter_value().integer_value
            byo = self.get_parameter('roi/bottom_offset').get_parameter_value().integer_value

            src = np.float32([[txro,tyo],[bxro,byo],[bxlo,byo],[txlo,tyo]])
            dst = np.float32([[self.width,0],[self.width,self.height],[0,self.height],[0,0]])
            self.M = cv2.getPerspectiveTransform(src,dst)
            self.Minv = cv2.getPerspectiveTransform(dst,src)
            self.do_on_set = False

        img = cv2.warpPerspective(img,self.M,img.shape[::-1],flags=cv2.INTER_LINEAR)
        return img

    def findLanePixels(self,img):
        histogram = np.sum(img[self.height//2:,:],axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        window_height = self.height // self.f_nwindows

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.f_nwindows):
            win_y_low = self.height - (window + 1) * window_height
            win_y_high = self.height - window * window_height
            win_xleft_low = leftx_current - self.f_margin
            win_xleft_high = leftx_current + self.f_margin
            win_xright_low = rightx_current - self.f_margin
            win_xright_high = rightx_current + self.f_margin
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > self.f_minpix:
                leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.f_minpix:        
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
        
        self.fit_poly(leftx, lefty, rightx, righty)

    def searchAroundPoly(self,img):
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        left_lane_inds = ((nonzerox > (self.left_lane_fit[0] * (nonzeroy ** 2) + self.left_lane_fit[1] * nonzeroy + self.left_lane_fit[2] - self.s_margin)) &
                          (nonzerox < (self.left_lane_fit[0] * (nonzeroy ** 2) + self.left_lane_fit[1] * nonzeroy + self.left_lane_fit[2] + self.s_margin)))
        right_lane_inds = ((nonzerox > (self.right_lane_fit[0] * (nonzeroy ** 2) + self.right_lane_fit[1] * nonzeroy + self.right_lane_fit[2] - self.s_margin)) &
                           (nonzerox < (self.right_lane_fit[0] * (nonzeroy ** 2) + self.right_lane_fit[1] * nonzeroy + self.right_lane_fit[2] + self.s_margin)))

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        self.fit_poly(leftx, lefty, rightx, righty)

    def fit_poly(self,leftx,lefty,rightx,righty):
        self.left_lane_fit = np.polyfit(lefty, leftx, 2)
        self.right_lane_fit = np.polyfit(righty, rightx, 2)

def main(args=None):
    rclpy.init(args=args)

    det_node = LaneDetection()

    rclpy.spin(det_node)

    rect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
