#!/usr/bin/python

# When the freenect launch file is executed, you
# must check on the driver/depth_registration flag within rqt_reconfigure

import rospy
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import AxesImage


def rgb_cb(msg, bridge, img_pub):
    global rgb_image
    
    # Let us load the disparity image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)   
    b,g,r = cv2.split(cv_image)       # get b,g,r
    rgb_image = cv2.merge([r,g,b])     # switch it to rgb

    merge_img(bridge, img_pub)

def disp_cb(msg, bridge, img_pub):
    global depth_image

    # Let us load the disparity image
    try:
        disparities = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)

    # And convert them into depth
    # From the doc of stereo_msgs/DisparityImage
    # For disparity d, the depth from the camera is Z = fT/d.
    depth_image = msg.f * msg.T / disparities
    Z_min, Z_max = msg.f * msg.T / msg.max_disparity, msg.f * msg.T / msg.min_disparity
    
    #merge_img(bridge, img_pub)

def merge_img(bridge, img_pub):
    merged = rgb_image.copy()
    # We filter out all the points that are at more than 1m from the sensor
    merged[depth_image >= 1.0] = 0
    img_pub.publish(bridge.cv2_to_imgmsg(merged, "rgb8"))


rgb_image = np.zeros((480, 640,3))
depth_image = np.zeros((480, 640))
bridge = CvBridge()

# Ros initialization
rospy.init_node("testDisparity")

image_pub = rospy.Publisher("image_out",Image, queue_size=1)
disparity_sub = rospy.Subscriber("disparity_in", DisparityImage, lambda img: disp_cb(img, bridge, image_pub))
rgb_sub = rospy.Subscriber("rgb_in", Image, lambda img: rgb_cb(img, bridge, image_pub))

# And the main loop simply triggers the update of the plot
rospy.spin()

