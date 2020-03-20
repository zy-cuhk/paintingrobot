#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
import scipy.io as io
import numpy as np
from math import *

def visualization_stl(num,position,color):
    marker=Marker()
    marker.header.frame_id="/world"
    marker.type=Marker.MESH_RESOURCE
    marker.mesh_resource="package://aubo_demo/stl_test/stl_document/0016.STL"
    marker.action=Marker.ADD
    marker.pose.orientation.w=1.0
    marker.scale.x=1.0
    marker.scale.y=1.0
    marker.scale.z=1.0
    marker.ns='arrow'
    marker.id=num
    marker.lifetime = rospy.Duration()

    marker.color.r=color.r
    marker.color.g=color.g
    marker.color.b=color.b
    marker.color.a=color.a

    marker.pose.position.x=position.x
    marker.pose.position.y=position.y
    marker.pose.position.z=position.z
    return marker


if __name__=="__main__":
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('markers')
    rate = rospy.Rate(10)

    marker=Marker()
    position = Point()
    position.x=0.0
    position.y=0.0
    position.z=0.0
    num=1
    color=ColorRGBA()
    color.r=1.0
    color.g=0.0
    color.b=0.0
    color.a=1.0
    marker=visualization_stl(num, position, color)
    # the_best_or_nothing=the_best_or_nothing+1

    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()


























