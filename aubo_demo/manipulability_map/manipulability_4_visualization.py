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

    min_position=-17
    side_length=len(manipulability_4)
    interval=1
    num=1
    planar_manipulability_4=np.zeros((side_length,side_length,side_length))
    markerarray = MarkerArray()
    position = Point()
    # for i in range(5,6,1):
    for i in range(side_length):
        # for j in range(5,6,1):
        for j in range(side_length):
            for k in range(side_length):
                position.x = min_position + i * interval
                position.y = min_position + j * interval
                position.z = min_position + k * interval

                if manipulability_4[i,j,k]>=0.30:
                    planar_manipulability_4[i,j,k]=1.0
                    data=manipulability_4[i,j,k]
                    color = compute_rgb(data)
                    marker = visualization_cube(num, position, color)
                    num = num + 1
                    markerarray.markers.append(marker)
    # waypoints_mat: xyz-rpy
    waypoints_mat=np.zeros((side_length,side_length,7))
    num=0
    for i in range(side_length):
        for j in range(side_length):
            if planar_manipulability_4[i,5,j]==1.0:
                waypoints_mat[i,j,0]=-0.850+i*0.050
                waypoints_mat[i,j,1]=-0.850+5*0.050
                waypoints_mat[i,j,2]=-0.850+j*0.050
                waypoints_mat[i,j,3]=-pi/2.0
                waypoints_mat[i,j,4]=0.0
                waypoints_mat[i,j,5]=0.0
                waypoints_mat[i,j,6]=1.0
                num=num+1
    # waypoints_list: xyz-rpy, with the flag==1, then we visualize it here:
    waypoints_list=np.zeros((num,6))
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
    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/scripts/waypoints_data.mat"
    io.savemat(mat_path,{"waypoints_list":waypoints_list})

    waypoints_data = io.loadmat(mat_path)
    waypoints_list = waypoints_data['waypoints_list']
    print("the length is:",len(waypoints_list))
    # for i in range(len(waypoints_list)):
    #     print(waypoints_list[i,0:6])

    while not rospy.is_shutdown():
        markerarray_pub.publish(markerarray)
        rate.sleep()

# ---------------------------------------------------------------------------------------------------------------------
# min_position = -0.5
# side_length = 10
# interval = 0.1
# data=0.3
# color = compute_rgb(data)
# print(color)
# if __name__ == '__main__':
#     pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
#     rospy.init_node("cubes")
#     rate=rospy.Rate(10)
#
#     side_length = 10
#     step = 1.0 / side_length
#
#     marker = Marker()
#     marker.header.frame_id = '/world'
#     marker.type=Marker.CUBE_LIST
#
#     marker.action = Marker.ADD
#     marker.pose.orientation.x = 0.0
#     marker.pose.orientation.y = 0.0
#     marker.pose.orientation.z = 0.0
#     marker.pose.orientation.w = 1.0
#
#     marker.scale.x = 0.2
#     marker.scale.y = 0.2
#     marker.scale.z = 0.2
#
#     marker.ns = 'cubes'
#     marker.id = 1
#     marker.color.r = 0.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0
#     marker.lifetime = rospy.Duration()
#
#     for i in range(side_length):
#         for j in range(side_length):
#             for k in range(side_length):
#                 point=Point()
#                 point.x=-0.5+i*step
#                 point.y=-0.5+j*step
#                 point.z=-0.5+k*step
#                 marker.points.append(point)
#     while not rospy.is_shutdown():
#         pub.publish(marker)
#         rate.sleep()

#             if data>=0.30:
#                 num = num + 1
#                 marker = visualization_marker(num, position, color)
#                 markerarray.markers.append(marker)
#
# for i in range(0,1,1):
# # for i in range(side_length):
#     for j in range(side_length):
#         for k in range(side_length):
#             position.x = min_position + i * interval
#             position.y = min_position + j * interval
#             position.z = min_position + k * interval
#             data=manipulability_1[i,j,k]
#             color = compute_rgb(data)
#             marker = visualization_marker(num, position, color)
#             num = num + 1
#             markerarray.markers.append(marker)
#
# for i in range(34,35,1):
# # for i in range(side_length):
#     for j in range(side_length):
#         for k in range(side_length):
#             position.x = min_position + i * interval
#             position.y = min_position + j * interval
#             position.z = min_position + k * interval
#             data=manipulability_1[i,j,k]
#             color = compute_rgb(data)
#             marker = visualization_marker(num, position, color)
#             num = num + 1
#             markerarray.markers.append(marker)

# for i in range(len(waypoints_mat)):
#     for j in range(len(waypoints_mat[0])):
#         if waypoints_mat[i,j,6]==1.0:
#             waypoints_list[num,0:6]=waypoints_mat[i,j,0:6]
#             num = num + 1

# the successful work we are doing here at this time
