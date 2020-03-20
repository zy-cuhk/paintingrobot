#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
import scipy.io as io
import numpy as np
from math import *

def compute_rgb(data):
    # print(data)
    color = ColorRGBA()
    if data<1.0/3:
        color.r=1.0
        color.g=3.0*data
        color.b=0.0
    if data>=1.0/3 and data<1.0/2:
        color.r = 3-6.0*data
        color.g = 1.0
        color.b = 0.0
    if data>=1.0/2 and data<2.0/3:
        color.r = 0.0
        color.g = 1.0
        color.b = 6.0*data-3
    if data>=2.0/3 and data<5.0/6:
        color.r = 0.0
        color.g = 5-6.0*data
        color.b = 1.0
    if data>=5.0/6 and data<=1.0:
        color.r = (900.0*data-750)/255.0
        color.g = 0.0
        color.b = 1.0
    color.a=1.0
    return color
def visualization_cube(num,position,color):
    marker=Marker()
    marker.header.frame_id="/world"
    marker.type=Marker.CUBE
    marker.action=Marker.ADD
    marker.pose.orientation.w=1.0
    marker.scale.x=0.04
    marker.scale.y=0.04
    marker.scale.z=0.04
    marker.ns='cube'
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
    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/manipulability_map/manipulability_data.mat"
    manipulability_data = io.loadmat(mat_path)
    manipulability_6 = manipulability_data['manipulability_6']

    manipulability_map=np.zeros((len(manipulability_6),len(manipulability_6)))
    interval=0.050
    height_num=int(floor((0.55-(-0.85))/interval))
    for i in range(len(manipulability_6)):
        for j in range(len(manipulability_6)):
            for k in range(height_num,height_num+1):
                if manipulability_6[i,j,k]>=0.20:
                    manipulability_map[i,j]=1.0
                else:
                    manipulability_map[i,j]=0.0

    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/cpp_planner/experiment_1/manipulability_map.mat"
    io.savemat(mat_path,{"manipulability_map":manipulability_map})

    # # # below code is applied for ROS RVIZ visualization
    # markerarray_pub=rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    # rospy.init_node('cubes')
    # rate=rospy.Rate(10)
    # markerarray=MarkerArray()
    # position=Point()
    # num=0
    # for i in range(len(manipulability_mat)):
    #     for j in range(len(manipulability_mat[i])):
    #         position.x=-0.850+0.050*i
    #         position.y=-0.850+0.050*j
    #         position.z=-0.850+0.050*height_num
    #         data=manipulability_mat[i,j]
    #         color=compute_rgb(data)
    #         marker=visualization_cube(num,position,color)
    #         num=num+1
    #         markerarray.markers.append(marker)
    # while not rospy.is_shutdown():
    #     markerarray_pub.publish(markerarray)
    #     rate.sleep()











