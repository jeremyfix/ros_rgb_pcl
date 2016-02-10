#!/usr/bin/python

# In this script, we demonstrate how to read a pointcloud XYZ RGB
# with python
# For fun, we generate an image
# This demonstration is to be used with gen_xyzrgb.py where we generate
# points with x,y in [0, 1]

import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import struct 
import numpy as np

def unpack_rgb(rgb_float, big_endian=False):
    # Get the bytes of the float rgb_float
    fmt = "f"
    if(big_endian):
        fmt = "!" + fmt
    packed = struct.pack(fmt , rgb_float)
    integers = [ord(c) for c in packed]
    return integers[0], integers[1], integers[2]
    
def pcl_cb(points_msg, bridge, image_pub):
    gen = pc2.read_points(points_msg, skip_nans=True)

    # The point cloud we use as an input is generating x,y in [0,1]
    # And the size (100 x 100) is actually choosen to get a filled image :)
    # because of the way we generate the data in gen_xyzrgb.py
    # by the way, this is just an example.
    height = 50
    width = 50
    cv_image = np.zeros((height,width,3), np.uint8)

    for x,y,z,rgb in gen:
        (r,g,b) = unpack_rgb(rgb)
        cv_image[height - 1 - round(y*height), round(x*width), :] = (r,g,b)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    

rospy.init_node("read_xyzrgb")
bridge = CvBridge()
image_pub = rospy.Publisher("image",Image, queue_size=1)
xyzrgb_pub = rospy.Subscriber("points", PointCloud2, lambda msg: pcl_cb(msg, bridge, image_pub))

rospy.spin()
