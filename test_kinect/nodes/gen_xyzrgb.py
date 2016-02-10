#!/usr/bin/python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from math import cos, pi
import struct 

def publish_points(points_pub):
    width, height = 50, 50
    rgb_points = PointCloud2()
    rgb_points.header.frame_id = "map"
    rgb_points.header.stamp = rospy.Time.now()
    rgb_points.width = width * height
    rgb_points.height  = 1
    rgb_points.fields.append(PointField(
        name = "x",offset = 0,
        datatype = PointField.FLOAT32,
        count = 1 ))
    rgb_points.fields.append(PointField(
        name = "y",offset = 4,
        datatype = PointField.FLOAT32,count = 1 ))
    rgb_points.fields.append(PointField(
        name = "z",offset = 8,
        datatype = PointField.FLOAT32,count = 1 ))
    rgb_points.fields.append(PointField(
        name = "rgb",offset = 16, 
        datatype = PointField.FLOAT32,count = 1 ))
    rgb_points.point_step = 32 
    rgb_points.row_step = rgb_points.point_step * rgb_points.width * rgb_points.height
    buffer = []
    phase = rospy.Time.now().to_sec() * 2.
    for y in range(height):
        for x in range(width):
            ptx = x/float(width);
            pty = y/float(height);
            ptz = 2.0 * (1. + cos(x/float(width) * 2.0 * pi + phase))/2.;
            rgb = (255*ptx, 255*pty, 255*ptz/2.)
            buffer.append(struct.pack('ffffBBBBIII',
                                      ptx,pty,ptz,1.0,
                                      rgb[0],rgb[1],rgb[2],0,
                                      0,0,0))
    rgb_points.data = "".join(buffer)
    points_pub.publish(rgb_points)


rospy.init_node("gen_xyzrgb")
xyzrgb_pub = rospy.Publisher("points", PointCloud2, queue_size=1)

rospy.loginfo("I'm generating a RGB pointcloud, watch me within rviz !")

while not rospy.is_shutdown():
    publish_points(xyzrgb_pub)
