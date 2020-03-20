#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
import scipy.io as io
import numpy as np
from math import *

if __name__ == "__main__":
    mat_path1 = "/home/zy/catkin_ws/src/aubo_robot/aubo_demo/cpp_planner/experiment_1/manipulability_map.mat"
    data1 = io.loadmat(mat_path1)
    manipulability_map = data1['manipulability_map']

    mat_path2 = "/home/zy/catkin_ws/src/aubo_robot/aubo_demo/cpp_planner/experiment_1/room_ceiling_waypoints_mat.mat"
    data2 = io.loadmat(mat_path2)
    room_ceiling_waypoints_mat = data2['room_ceiling_waypoints_mat']

    # layers_num = int(ceil(float(len(room_ceiling_waypoints_mat))/float(len(manipulability_map))))
    # for i in range(1):
    # # for i in range(layers_num):
    #     index_min=i*len(manipulability_map)
    #     index_max=(i+1)*len(manipulability_map)
    #     if index_max>len(room_ceiling_waypoints_mat):
    #         index_max=len(room_ceiling_waypoints_mat)
    #     layers_mat=np.zeros((index_max-index_min,len(room_ceiling_waypoints_mat[0])))
    #     for j in range(index_max-index_min):
    #         for k in range(len(room_ceiling_waypoints_mat[0])):
    #             layers_mat[j,k]=room_ceiling_waypoints_mat[i*len(manipulability_map)+j,k]

    interval=0.050
    layers_num = int(ceil(float(len(room_ceiling_waypoints_mat[0]))/float(len(manipulability_map[0]))))
    for i in range(1):
    # for i in range(layers_num):
        # divide the whole map into several layers
        index_min=i*len(manipulability_map[0])
        index_max=(i+1)*len(manipulability_map[0])
        if index_max>len(room_ceiling_waypoints_mat[0]):
            index_max=len(room_ceiling_waypoints_mat[0])
        layers_mat=np.zeros((len(room_ceiling_waypoints_mat),index_max-index_min))
        for j in range(len(room_ceiling_waypoints_mat)):
            for k in range(index_max-index_min):
                layers_mat[j,k]=room_ceiling_waypoints_mat[j,i*len(manipulability_map[0])+k]
        # compute the distance between most left and most right critical points of each layer
        col=np.zeros(len(layers_mat[0]))
        m=0
        while(1):
            col[:]=layers_mat[m,:]
            if len(np.where(col==0)[0])!=0 or m==len(layers_mat):
                break
            else:
                m=m+1
        distance=len(layers_mat)-m+1
        print(distance)
        # find the most left critical point
        critical_points=layers_mat[m,np.where(col==0)[0][0]]
        point_x=(m+1)*interval
        point_y=(np.where(col==0)[0][0]+i*len(manipulability_map[0])+1)*interval
        # print(m)
        # print(np.where(col==0)[0][0])
        # print(point_x)
        # print(point_y)
        # determine the positions of sampling points
        layers_mat1=layers_mat
        distance=np.zeros((len(manipulability_map),len(manipulability_map)))
        for m1 in range(0,1):
            for m2 in range(2,3):
        # for m1 in range(len(manipulability_map)):
            # for m2 in range(len(manipulability_map)):
                placementposition_sampling_x=point_x+m1*interval-0.850
                placementposition_sampling_y=point_y+m2*interval-0.850

                # adjust the manipulability_map and combine with the layer_mat
                manipulability_map_sampling_xmin=placementposition_sampling_x-0.850
                manipulability_map_sampling_xmax=placementposition_sampling_x+0.850
                manipulability_map_sampling_ymin=placementposition_sampling_y-0.850
                manipulability_map_sampling_ymax=placementposition_sampling_y+0.850

                manipulability_map_sampling_xbasis=manipulability_map_sampling_xmin
                manipulability_map_sampling_ybasis=manipulability_map_sampling_ymin

                if manipulability_map_sampling_xmin<=0.0:
                    manipulability_map_sampling_xmin=0.050
                if manipulability_map_sampling_ymin<=0.0:
                    manipulability_map_sampling_ymin=0.050
                if manipulability_map_sampling_xmax>len(room_ceiling_waypoints_mat)*0.050:
                    manipulability_map_sampling_xmax=len(room_ceiling_waypoints_mat)*0.050
                if manipulability_map_sampling_ymax>len(room_ceiling_waypoints_mat[0])*0.050:
                    manipulability_map_sampling_ymax=len(room_ceiling_waypoints_mat[0])*0.050
                print(manipulability_map_sampling_ymin)
                print(manipulability_map_sampling_ybasis)
                manipulability_map_sampling_xmin_index=int((manipulability_map_sampling_xmin-manipulability_map_sampling_xbasis)/interval)
                manipulability_map_sampling_xmax_index=int((manipulability_map_sampling_xmax-manipulability_map_sampling_xbasis)/interval)+1
                manipulability_map_sampling_ymin_index=int((manipulability_map_sampling_ymin-manipulability_map_sampling_ybasis)/interval)
                manipulability_map_sampling_ymax_index=int((manipulability_map_sampling_ymax-manipulability_map_sampling_ybasis)/interval)+1

                layers_mat_xmin_index=int((manipulability_map_sampling_xmin-0.050)/interval-i)
                layers_mat_xmax_index=int((manipulability_map_sampling_xmax-0.050)/interval-i)+1
                layers_mat_ymin_index=int((manipulability_map_sampling_ymin-0.050)/interval)
                layers_mat_ymax_index=int((manipulability_map_sampling_ymax-0.050)/interval)+1

                print(manipulability_map_sampling_xmin_index)
                print(manipulability_map_sampling_xmax_index)
                print(manipulability_map_sampling_ymin_index)
                print(manipulability_map_sampling_ymax_index)

                print(layers_mat_xmin_index)
                print(layers_mat_xmax_index)
                print(layers_mat_ymin_index)
                print(layers_mat_ymax_index)

                print(manipulability_map[manipulability_map_sampling_xmin_index:manipulability_map_sampling_xmax_index,manipulability_map_sampling_ymin_index:manipulability_map_sampling_ymax_index])
                print(layers_mat1[layers_mat_xmin_index:layers_mat_xmax_index,layers_mat_ymin_index:layers_mat_ymax_index])
                mat1=manipulability_map[manipulability_map_sampling_xmin_index:manipulability_map_sampling_xmax_index,manipulability_map_sampling_ymin_index:manipulability_map_sampling_ymax_index]
                mat2=layers_mat1[layers_mat_xmin_index:layers_mat_xmax_index,layers_mat_ymin_index:layers_mat_ymax_index]
                mat1=np.logical_or(mat1,mat2)
                mat1=np.array(mat1).astype(int)
                print(mat1)
                layers_mat1[layers_mat_xmin_index:layers_mat_xmax_index,layers_mat_ymin_index:layers_mat_ymax_index]=mat1[:,:]

                # for the sampling points, recompute the distance between critical points
                col=np.zeros(len(layers_mat1[0]))
                m=0
                while (1):
                    col[:]=layers_mat1[m,:]
                    if len(np.where(col==0)[0])!=0 or m==len(layers_mat1):
                        break
                    else:
                        m=m+1
                distance[m1,m2]=len(layers_mat1)-m+1
                print(distance[m1,m2])
                # select the robot placement positions with the most decresing distance
        min_distance=distance.min()
                # readjust the manipulability map and combine with the layer_mat

    # print(layers_mat[0,3])
    # print(manipulability_map[34,34])
    # a=np.ones((3,4))
    # b=np.zeros((2,2))
    # c=np.logical_or(a[1:3,0:2],b)
    # c=np.array(c).astype(int)
    # print(c)

    # print(len(room_ceiling_waypoints_mat[0]))
    # print(len(room_ceiling_waypoints_mat))
    # print(len(layers_mat))
    # print(len(layers_mat[0]))
    # print(layers_mat[0,:])
    # x=np.array([1,2,3,1])
    # print(len(np.where(x==0)[0]))
    # print(len(room_ceiling_waypoints_mat)/len(manipulability_map))
    # print(len(room_ceiling_waypoints_mat))
    # print(len(manipulability_map))
    # print(layers_num)
