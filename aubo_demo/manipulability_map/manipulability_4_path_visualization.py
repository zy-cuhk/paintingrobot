#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,Pose,PoseStamped,Quaternion
from nav_msgs.msg import Path
from aubo_kinematics import *
from std_msgs.msg import ColorRGBA
import scipy.io as io
import numpy as np
from math import *
from numpy import matlib

class pose2mat():
    def __init__(self):
        self.a=3
    def rotx(self,q):
        cq=cos(q)
        sq=sin(q)
        R =[[1, 0, 0],
            [0, cq, -sq],
            [0, sq, cq]]
        return R
    def roty(self,q):
        cq = cos(q)
        sq = sin(q)
        R=[[cq, 0, sq],
           [0, 1, 0],
           [-sq,0,cq]]
        return  R
    def rotz(self,q):
        cq=cos(q)
        sq=sin(q)
        R=[[cq, -sq, 0],
           [sq,  cq, 0],
           [ 0,   0, 1]]
        return R
    def rpy2r(self,q):
        roll=q[0]
        pitch=q[1]
        yaw=q[2]
        Rz = self.rotz(yaw)
        Ry = self.roty(pitch)
        Rx = self.rotx(roll)
        R1=np.dot(Rz, Ry)
        R=np.dot(R1,Rx)
        return R
    def rpy2tr(self,q):
        T = np.matlib.identity(4, dtype=float)
        R=self.rpy2r(q)
        T[0:3, 0:3] = R
        # T[0:3,0:3]=np.array(R)
        return T
    def tran2r(self,p):
        r = np.array(p).reshape((3, 1))
        return r
    def tran2tr(self,p):
        T = np.matlib.identity(4, dtype=float)
        T[0:3, 3] = np.array(p).reshape((3, 1))
        return T
    def mat4x4(self,p,q):
        T = np.matlib.identity(4, dtype=float)
        rot = self.rpy2r(q)
        tran = self.tran2r(p)
        T[0:3, 0:3] = rot
        T[0:3, 3] = tran
        return T

if __name__=="__main__":
    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/scripts/waypoints_data.mat"
    waypoints_data = io.loadmat(mat_path)
    waypoints_list = waypoints_data['waypoints_list']
    print("the length is:", len(waypoints_list))

    path_pub=rospy.Publisher("visualization_path", Path, queue_size=10)
    rospy.init_node('path')
    rate=rospy.Rate(10)

    path = Path()
    current_time = rospy.Time.now()
    path.header.frame_id = "/world"
    path.header.stamp=current_time

    pose_mat = pose2mat()
    ak47=Aubo_kinematics()
    ref_angle=[0.9467347960019694, 1.4850652551594532, -0.36718623153275676, 1.289341166897584, -2.1948578575878237, 0.0]
    waypoints_joints_value=np.zeros((len(waypoints_list),6))
    count=0
    # for i in range(1):
    for i in range(len(waypoints_list)):
        tran=waypoints_list[i,0:3]
        rpy=waypoints_list[i,3:6]
        T = pose_mat.mat4x4(tran, rpy)
        T1 = np.array(T).reshape((1,16))
        tt = (T1[0]).tolist()
        mat=ak47.GetInverseResult(tt, ref_angle)
        if len(mat)==0:
            count=count+1
        waypoints_joints_value[i,0:6]=mat
        ref_angle=mat
        print("mat=",mat)
    print("count=",count)

    mat_path="/home/zy/catkin_ws/src/spraying_robot/aubo_driver/libpyauboi5-v1.2.2.x64/waypoints_joint_value.mat"
    io.savemat(mat_path,{"waypoints_joints_value":waypoints_joints_value})

    # print ak47.GetInverseResult(tt, ref_angle)
    # joint_rad=[-0.1376507696685616, -0.4807023051897623, -2.0148282757396903, -1.4938294927279987, -1.4230236674811794, -0.00034126231777253935]
    # print ak47.aubo_forward(ak47.rad_to_degree(joint_rad))

    while not rospy.is_shutdown():
        for i in range(len(waypoints_list)):
            current_time=rospy.Time.now()
            this_pose_stamped=PoseStamped()
            this_pose_stamped.pose.position.x=waypoints_list[i,0]
            this_pose_stamped.pose.position.y=waypoints_list[i,1]
            this_pose_stamped.pose.position.z=waypoints_list[i,2]

            this_pose_stamped.header.stamp=current_time
            this_pose_stamped.header.frame_id="/world"
            path.poses.append(this_pose_stamped)
            path_pub.publish(path)
            rate.sleep()




    # x=0.0
    # y=0.0
    # th=0.0
    # vx=0.1
    # vy=0.1
    # vth=0.1
    # delta_x = vx * dt
    # delta_y = vy * dt
    # delta_x=(vx*cos(th)-vy*sin(th))*dt
    # delta_y=(vy*sin(th)+vy*cos(th))*dt
    # delta_th = vth * dt







