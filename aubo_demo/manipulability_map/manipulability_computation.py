#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import *
import numpy.matlib
import numpy as np
import scipy.io as io

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from time import clock

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

class Aubo_kinematics():
    def __init__(self):
        self.a2 = 0.408
        self.a3 = 0.376
        self.d1 = 0.122
        self.d2 = 0.1215
        self.d5 = 0.1025
        self.d6 = 0.094
        self.ZERO_THRESH = 1e-4
        self.ARM_DOF = 6

        self.robot = self.init_robot("/home/zy/catkin_ws/src/aubo_robot/aubo_description/urdf/aubo_i5.urdf")
        self.kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "wrist3_Link")
        self.q = [0, 0, 0, 0, 0, 0]

    def degree_to_rad(self, q):
        temp = []
        for i in range(len(q)):
            temp.append(q[i] * pi / 180)
        return temp

    def antiSinCos(self, sA, cA):

        eps = 1e-8
        angle = 0
        if ((abs(sA) < eps) and (abs(cA) < eps)):
            return 0

        if (abs(cA) < eps):
            angle = pi / 2.0 * self.SIGN(sA)
        elif (abs(sA) < eps):

            if (self.SIGN(cA) == 1):
                angle = 0
            else:
                angle = pi

        else:

            angle = atan2(sA, cA)

        return angle

    def SIGN(self, x):
        return (x > 0) ^ (x < 0)

    def aubo_forward(self, q):
        q = self.degree_to_rad(q)
        T = []
        for i in range(16):
            T.append(0)
        # print q
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]
        C1 = cos(q1)
        C2 = cos(q2)
        C4 = cos(q4)
        C5 = cos(q5)
        C6 = cos(q6)
        C23 = cos(q2 - q3)
        C234 = cos(q2 - q3 + q4)
        C2345 = cos(q2 - q3 + q4 - q5)
        C2345p = cos(q2 - q3 + q4 + q5)
        S1 = sin(q1)
        S2 = sin(q2)
        S4 = sin(q4)
        S5 = sin(q5)
        S6 = sin(q6)
        S23 = sin(q2 - q3)
        S234 = sin(q2 - q3 + q4)

        T[0] = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6)
        T[1] = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6)
        T[2] = C5 * S1 + C1 * C234 * S5
        T[3] = (self.d2 + C5 * self.d6) * S1 - C1 * (
                    self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[4] = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6
        T[5] = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6
        T[6] = -C1 * C5 + C234 * S1 * S5
        T[7] = -C1 * (self.d2 + C5 * self.d6) - S1 * (
                    self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[8] = C5 * C6 * S234 + C234 * S6
        T[9] = C234 * C6 - C5 * S234 * S6
        T[10] = S234 * S5
        T[11] = self.d1 + self.a2 * C2 + self.a3 * C23 + self.d5 * C234 + self.d6 * C2345 / 2 - self.d6 * C2345p / 2
        T[12] = 0
        T[13] = 0
        T[14] = 0
        T[15] = 1
        return T
    def array_to_List(self,T_array):
        """
        :param T_array:
        :return:
        """
        T_list=T_array.tolist()
        # print(T_list)
        T_result=[]
        for i in range(len(T_list)):
            for j in range(len(T_list[0])):
                T_result.append(T_list[i][j])
        return T_result
    def aubo_inverse(self, T):
        q_reslut_dic = {}
        q_reslut = []
        singularity = False

        num_sols = 0
        nx = T[0]
        ox = T[1]
        ax = T[2]
        px = T[3]

        ny = T[4]
        oy = T[5]
        ay = T[6]
        py = T[7]
        nz = T[8]
        oz = T[9]
        az = T[10]
        pz = T[11]

        # //////////////////////// shoulder rotate joint (q1) //////////////////////////////
        q1 = [0, 0]

        A1 = self.d6 * ay - py
        B1 = self.d6 * ax - px
        R1 = A1 * A1 + B1 * B1 - self.d2 * self.d2

        if R1 < 0.0:
            return num_sols
        else:
            R12 = sqrt(R1)
            q1[0] = self.antiSinCos(A1, B1) - self.antiSinCos(self.d2, R12)
            q1[1] = self.antiSinCos(A1, B1) - self.antiSinCos(self.d2, -R12)
            for i in range(len(q1)):

                while q1[i] > pi:
                    q1[i] -= 2 * pi
                while q1[i] < -pi:
                    q1[i] += 2 * pi

        # ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
        q5 = [[0, 0], [0, 0]]

        for i in range(len(q5)):
            C1 = cos(q1[i])
            S1 = sin(q1[i])
            B5 = -ay * C1 + ax * S1
            M5 = (-ny * C1 + nx * S1)
            N5 = (-oy * C1 + ox * S1)
            R5 = sqrt(M5 * M5 + N5 * N5)

            q5[i][0] = self.antiSinCos(R5, B5)
            q5[i][1] = self.antiSinCos(-R5, B5)

        # ////////////////////////////////////////////////////////////////////////////////

        # ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
        q6 = 0

        q3 = [0, 0]
        q2 = [0, 0]
        q4 = [0, 0]
        for i in range(len(q3)):

            for j in range(len(q3)):

                # // wrist 3 joint (q6) //
                C1 = cos(q1[i])
                S1 = sin(q1[i])
                S5 = sin(q5[i][j])

                A6 = (-oy * C1 + ox * S1)
                B6 = (ny * C1 - nx * S1)

                if fabs(S5) < self.ZERO_THRESH:  # //the condition is only dependent on q1

                    singularity = True
                    break
                else:
                    q6 = self.antiSinCos(A6 * S5, B6 * S5)

                # /////// joints (q3,q2,q4) //////
                C6 = cos(q6)
                S6 = sin(q6)

                pp1 = C1 * (ax * self.d6 - px + self.d5 * ox * C6 + self.d5 * nx * S6) + S1 * (
                            ay * self.d6 - py + self.d5 * oy * C6 + self.d5 * ny * S6)
                pp2 = -self.d1 - az * self.d6 + pz - self.d5 * oz * C6 - self.d5 * nz * S6
                B3 = (pp1 * pp1 + pp2 * pp2 - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3)

                if ((1 - B3 * B3) < self.ZERO_THRESH):
                    singularity = True
                    continue
                else:
                    Sin3 = sqrt(1 - B3 * B3)
                    q3[0] = self.antiSinCos(Sin3, B3)
                    q3[1] = self.antiSinCos(-Sin3, B3)

                for k in range(len(q3)):

                    C3 = cos(q3[k])
                    S3 = sin(q3[k])
                    A2 = pp1 * (self.a2 + self.a3 * C3) + pp2 * (self.a3 * S3)
                    B2 = pp2 * (self.a2 + self.a3 * C3) - pp1 * (self.a3 * S3)

                    q2[k] = self.antiSinCos(A2, B2)
                    C2 = cos(q2[k])
                    S2 = sin(q2[k])

                    A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6)
                    B4 = oz * C6 + nz * S6
                    A41 = pp1 - self.a2 * S2
                    B41 = pp2 - self.a2 * C2

                    q4[k] = self.antiSinCos(A4, B4) - self.antiSinCos(A41, B41)
                    while (q4[k] > pi):
                        q4[k] -= 2 * pi
                    while (q4[k] < -pi):
                        q4[k] += 2 * pi
                    q_reslut = [q1[i], q2[k], q3[k], q4[k], q5[i][j], q6]

                    q_reslut_dic.update({num_sols: q_reslut})
                    num_sols += 1
        if num_sols!=0:
            # print(num_sols)
            return q_reslut_dic,num_sols
        else:
            return False

    def List_Frobenius_Norm(self, list_a, list_b):
        new_list = []
        if len(list_a) == len(list_b):
            for i in range(len(list_a)):
                new_list.append(abs(list_a[i] - list_b[i]) ** 2)
        else:
            print("please make sure the list has the same length")

        return sqrt(self.sum_list(new_list))

    def sum_list(self, list_data):
        sum_data = 0
        for i in range(len(list_data)):
            sum_data += list_data[i]
        return sum_data

    def selectIK(self, q_sols, AngleLimit):
        q_sols_selected = {}
        N = len(q_sols)
        # print "selectIK ---N---",N
        if (N == 0):
            return False, {}
        num = 0
        valid = True
        for i in range(N):

            valid = True;
            for j in range(self.ARM_DOF):
                # drop greater than offical degree
                if (q_sols[i][j] > AngleLimit[j][1] or q_sols[i][j] < AngleLimit[j][0]):
                    valid = False
                    break

            # delete the sols about joint angular increase 2*pi greater than the legal angular
            if (valid):
                temp = q_sols[i]
                temp_1 = q_sols[i]
                for j in range(self.ARM_DOF):
                    temp[j] += 2 * pi
                    temp_1[j] -= 2 * pi
                    if temp[j] > AngleLimit[j][1] or temp_1[j] < AngleLimit[j][0]:
                        valid = False
                        break
                if valid:
                    q_sols_selected.update({num: q_sols[i]})
                    num += 1

        num_sols = num

        if (num > 0):
            return True, q_sols_selected
        else:
            return False, {}

    def init_robot(self, filename):
        robot = URDF.from_xml_file(filename)
        return robot

    def get_jacobian(self, q_list):
        self.q = q_list
        J = self.kdl_kin.jacobian(self.q)
        return J
    def compute_manipulability(self,flag, q_sols):
        if flag!=False:
            manipulability_value=np.zeros(len(q_sols))
            for i in range(manipulability_value.size):
                self.q=q_sols[i]
                J = self.kdl_kin.jacobian(self.q)
                J_mat = np.dot(J, J.T)
                manipulability_value[i] = sqrt(np.linalg.det(J_mat))
                # print("single manipulability value:",manipulability_value[i])
        else:
            manipulability_value=np.zeros(1)

        # average_manipulability_value=sum(manipulability_value)/len(manipulability_value)
        average_manipulability_value=np.min(manipulability_value)
        return average_manipulability_value

def main():
    t1=clock()
    #-----------------------------------------------------------------------------------------------------------------
    # step one: form a matrix which saves the rpy, the target is a 5d matrix: xyz-6-3
    radius=850*0.001
    grid_size=50*0.001
    xyz_num=int(2*radius/grid_size+1)
    print(xyz_num)
    mat1=np.arange(xyz_num*xyz_num*xyz_num*6*3,dtype=float).reshape([xyz_num,xyz_num,xyz_num,6,3])
    manipulability_map=np.arange(xyz_num*xyz_num*xyz_num*6,dtype=float).reshape([xyz_num,xyz_num,xyz_num,6])
    relative_manipulability_map=np.arange(xyz_num*xyz_num*xyz_num*6,dtype=float).reshape([xyz_num,xyz_num,xyz_num,6])
    maxq=175.0/180*pi
    AngleLimit = [(-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq)]

    for i in range(xyz_num):
        for j in range(xyz_num):
            for k in range(xyz_num):
                for m in range(6):
                    if m==0:
                        mat1[i,j,k,m,0:3]=np.array([-pi/2,0,pi/2])
                    elif m==1:
                        mat1[i,j,k,m,0:3]=np.array([pi/2,0,pi/2])
                    elif m==2:
                        mat1[i,j,k,m,0:3] = np.array([pi/2,0,0])
                    elif m==3:
                        mat1[i,j,k,m,0:3]=np.array([-pi/2,0,0])
                    elif m==4:
                        mat1[i,j,k,m,0:3] = np.array([pi,0,0])
                    elif m==5:
                        mat1[i,j,k,m,0:3] = np.array([0,0,0])
    #-------------------------------------------------------------------------------------------------------------------
    # step two: form a matrix which uses rpy to form the robot pose matrix
    pose_mat=pose2mat()
    aubo5=Aubo_kinematics()
    for i in range(xyz_num):
        for j in range(xyz_num):
            for k in range(xyz_num):
                for m in range(6):
                    x0=-radius+i*grid_size
                    y0=-radius+j*grid_size
                    z0=-radius+k*grid_size
                    if x0**2+y0**2+z0**2-radius**2>0:
                        manipulability_map[i,j,k,m]=0
                    else:
                        p=np.array([x0,y0,z0],dtype=float)
                        q=mat1[i,j,k,m,0:3]
                        T=pose_mat.mat4x4(p,q)
                        T1=np.array(T).reshape((1,16))
                        tt=(T1[0]).tolist()
                        if aubo5.aubo_inverse(tt)!=False:
                            q_sols_all,num_sols=aubo5.aubo_inverse(tt)
                            if  q_sols_all!=False:
                                if (len(q_sols_all)!=0):
                                    ret2, q_sols_inlimit=aubo5.selectIK(q_sols_all,AngleLimit)

                                    manipulability_map[i,j,k,m]=aubo5.compute_manipulability(ret2, q_sols_inlimit)
                        else:
                            manipulability_map[i,j,k,m]=0
                    # print('manipulability value:',manipulability_map[i,j,k,m])

    max_manipulability=np.max(manipulability_map)
    for i in range(xyz_num):
        for j in range(xyz_num):
            for k in range(xyz_num):
                for m in range(6):
                    relative_manipulability_map[i,j,k,m]=manipulability_map[i,j,k,m]/max_manipulability

    relative_manipulability_map_1 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])
    relative_manipulability_map_2 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])
    relative_manipulability_map_3 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])
    relative_manipulability_map_4 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])
    relative_manipulability_map_5 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])
    relative_manipulability_map_6 = np.arange(xyz_num * xyz_num * xyz_num, dtype=float).reshape([xyz_num, xyz_num, xyz_num])

    for i in range(xyz_num):
        for j in range(xyz_num):
            for k in range(xyz_num):
                for m in range(6):
                    if m==0:
                        relative_manipulability_map_1[i,j,k]=relative_manipulability_map[i, j, k, m]
                    elif m==1:
                        relative_manipulability_map_2[i,j,k]=relative_manipulability_map[i, j, k, m]
                    elif m==2:
                        relative_manipulability_map_3[i, j, k] = relative_manipulability_map[i, j, k, m]
                    elif m==3:
                        relative_manipulability_map_4[i, j, k] = relative_manipulability_map[i, j, k, m]
                    elif m==4:
                        relative_manipulability_map_5[i, j, k] = relative_manipulability_map[i, j, k, m]
                    elif m==5:
                        relative_manipulability_map_6[i, j, k] = relative_manipulability_map[i, j, k, m]

    t2 = clock()
    t = t2 - t1
    print("the consumed time is:{" + str(t) + "} seconds")
    max_relative_manipulability=np.max(relative_manipulability_map)
    min_relative_manipulability=np.min(relative_manipulability_map)
    print("the maximum manipulability is:", max_relative_manipulability)
    print("the minimum manipulability is:", min_relative_manipulability)

    max_relative_manipulability_1=np.max(relative_manipulability_map_1)
    min_relative_manipulability_1=np.min(relative_manipulability_map_1)
    print("the maximum manipulability_1 is:", max_relative_manipulability_1)
    print("the minimum manipulability_1 is:", min_relative_manipulability_1)

    max_relative_manipulability_2=np.max(relative_manipulability_map_2)
    min_relative_manipulability_2=np.min(relative_manipulability_map_2)
    print("the maximum manipulability_2 is:", max_relative_manipulability_2)
    print("the minimum manipulability_2 is:", min_relative_manipulability_2)

    max_relative_manipulability_3=np.max(relative_manipulability_map_3)
    min_relative_manipulability_3=np.min(relative_manipulability_map_3)
    print("the maximum manipulability_3 is:", max_relative_manipulability_3)
    print("the minimum manipulability_3 is:", min_relative_manipulability_3)

    max_relative_manipulability_4 = np.max(relative_manipulability_map_4)
    min_relative_manipulability_4 = np.min(relative_manipulability_map_4)
    print("the maximum manipulability_4 is:", max_relative_manipulability_4)
    print("the minimum manipulability_4 is:", min_relative_manipulability_4)

    max_relative_manipulability_5 = np.max(relative_manipulability_map_5)
    min_relative_manipulability_5 = np.min(relative_manipulability_map_5)
    print("the maximum manipulability_5 is:", max_relative_manipulability_5)
    print("the minimum manipulability_5 is:", min_relative_manipulability_5)

    max_relative_manipulability_6 = np.max(relative_manipulability_map_6)
    min_relative_manipulability_6 = np.min(relative_manipulability_map_6)
    print("the maximum manipulability_6 is:", max_relative_manipulability_6)
    print("the minimum manipulability_6 is:", min_relative_manipulability_6)

    mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/scripts/manipulability_data"
    # mat_1=np.arange(4).reshape((2,2))
    # mat_2=np.arange(6)
    io.savemat(mat_path,{"manipulability_1":relative_manipulability_map_1,"manipulability_2":relative_manipulability_map_2,
                         "manipulability_3":relative_manipulability_map_3,"manipulability_4":relative_manipulability_map_4,
                         "manipulability_5":relative_manipulability_map_5,"manipulability_6":relative_manipulability_map_6})

if __name__=="__main__":
        main()




    # compute_manipulability(self, flag, q_sols):
    # compute_manipulability(self,flag, q_sols)
    # maxq=175.0/180*pi
    # AngleLimit = [(-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq)]
    #
    # q = mat1[0, 0, 0, 0, 0:3]
    # p = np.zeros(3, dtype=float)
    # p[0] = (-radius + 10 * grid_size) * 0.001
    # p[1] = (-radius + 10 * grid_size) *if ((len(q_sols_inlimit)!= 0) and (True == ret2)):
    #         print(" find solution choose  ")
    #         # return q_sols_inlimit
    #         for i in q_sols_inlimit:
    #             print("num:"+str(i)+''+"suitable sols",q_sols_inlimit[i])
    #     else:
    #         print("no valid sols ") 0.001
    # p[2] = (-radius + 10 * grid_size) * 0.001
    # T = s.mat4x4(p, q)
    # T1 = np.array(T).reshape((1, 16))
    # tt = T1[0]
    #
    # q_sols_all, num_sols = aubo5.aubo_inverse(tt)
    # if (len(q_sols_all) != 0):
    #     for i in q_sols_all:
    #         print("num:" + str(i) + ' ' + "sols", q_sols_all[i])
    #     # remove not in limited data
    #     ret2, q_sols_inlimit = aubo5.selectIK(q_sols_all, AngleLimit)
    #     # print "q_sols_inlimit",q_sols_inlimit
    #     if ((len(q_sols_inlimit)!= 0) and (True == ret2)):
    #         print(" find solution choose  ")
    #         # return q_sols_inlimit
    #         for i in q_sols_inlimit:
    #             print("num:"+str(i)+''+"suitable sols",q_sols_inlimit[i])
    #     else:
    #         print("no valid sols ")
    # for i in range(xyz_num):
    #     for j in range(xyz_num):
    #         for k in range(xyz_num):
    #             for m in range(6):
    #                 roll=mat1[i,j,k,m,0]
    #                 pitch=mat1[i,j,k,m,1]
    #                 yaw=mat1[i,j,k,m,2]
    # print(" find solution choose  ")
    # # return q_sols_inlimit
    # for i in q_sols_inlimit:
    #     print("num:" + str(i) + '' + "suitable sols", q_sols_inlimit[i])

    # if ((len(q_sols_inlimit) != 0) and (True == ret2)):
    #     manipulability_map[i,j,k,m]=1
    #     for i in range(len(q_sols_inlimit)):
    #         print("q_sols_inlimit:", q_sols_inlimit[i])
    # else:
    #     manipulability_map[i,j,k,m]=0
    #     print("no valid sols ")