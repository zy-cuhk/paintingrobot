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
    marker.scale.x=1.0
    marker.scale.y=1.0
    marker.scale.z=1.0
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

def visualization_arrow(num,position,color):
    marker=Marker()
    marker.header.frame_id="/world"
    marker.type=Marker.ARROW
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
    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/scripts/manipulability_data.mat"
    manipulability_data = io.loadmat(mat_path)
    manipulability_1 = manipulability_data['manipulability_1']
    manipulability_2 = manipulability_data['manipulability_2']
    manipulability_3 = manipulability_data['manipulability_3']
    manipulability_4 = manipulability_data['manipulability_4']
    manipulability_5 = manipulability_data['manipulability_5']
    manipulability_6 = manipulability_data['manipulability_6']
    markerarray_pub=rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rospy.init_node('cubes')
    rate=rospy.Rate(10)

    # waypoints_mat: xyz-rpy
    count_num=0
    waypoints_mat=np.zeros((len(manipulability_6),len(manipulability_6),7))
    for i in range(len(manipulability_6)):
        for j in range(len(manipulability_6)):
            if manipulability_6[i,j,28]>=0.30:
                waypoints_mat[i,j,0]=-0.850+i*0.050
                waypoints_mat[i,j,1]=-0.850+j*0.050
                waypoints_mat[i,j,2]=-0.850+28*0.050
                waypoints_mat[i,j,3]=0.0
                waypoints_mat[i,j,4]=0.0
                waypoints_mat[i,j,5]=0.0
                waypoints_mat[i,j,6]=1.0
                count_num=count_num+1
    # waypoints_list: xyz-rpy, with the flag==1, then we visualize it here:
    print("count_num=:",count_num)
    waypoints_list=np.zeros((count_num,6))
    num=0
    flag1=0
    for i in range(len(waypoints_mat)):
        flag1_before=flag1
        flag2=0
        for j in range(len(waypoints_mat[0])):
            if waypoints_mat[i,j,6]==1.0:
                flag2=1
        if flag2==1:
            flag1=flag1+1
        if flag1>flag1_before:
            if (flag1)%2==1:
                for j in range(len(waypoints_mat[0])):
                    if waypoints_mat[i, j, 6] == 1.0:
                        waypoints_list[num, 0:6] = waypoints_mat[i, j, 0:6]
                        num = num + 1
            elif (flag1)%2==0:
                for j in range(len(waypoints_mat[0])):
                    if waypoints_mat[i,-j-1, 6] == 1.0:
                        waypoints_list[num, 0:6] = waypoints_mat[i, -j-1, 0:6]
                        num = num + 1
    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/scripts/waypoints_data_2.mat"
    io.savemat(mat_path,{"waypoints_list":waypoints_list})
    waypoints_data = io.loadmat(mat_path)
    waypoints_list = waypoints_data['waypoints_list']
    # for i in range(len(waypoints_list)):
    #    print(waypoints_list[i,0:6])

    # the below ones must be done after lunch
    # visualize the manipulability matrix
    markerarray = MarkerArray()
    position = Point()
    num = 0
    for i in range(len(manipulability_6)):
        for j in range(len(manipulability_6[0])):
            for k in range(28,29):
            # for k in range(len(manipulability_6[0, 0])):
                position.x = (-17 + i)
                position.y = (-17 + j)
                position.z = (-17 + k)
                if manipulability_6[i, j, k]>=0.30:
                    data = manipulability_6[i, j, k]
                    color = compute_rgb(data)
                    marker = visualization_cube(num, position, color)
                    num = num + 1
                    markerarray.markers.append(marker)
    while not rospy.is_shutdown():
        markerarray_pub.publish(markerarray)
        rate.sleep()


