#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from math import *
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot
import scipy.io as io

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,Pose
from std_msgs.msg import ColorRGBA

# house_stl_reading/house_stl_matplot-->room_vertices_generation-->room_planes_gereration-->room_planes_selection
# -->room_planes_matplot/room_planes_rvizplot-->room_planes_paths_generation-->room_plane_paths_matplot/room_plane_paths_rvizplot
# 1. house_stl_reading, input: stl document, output: facet,vertices,norm_vectors
# 2. room_vertices_generation, input: vertices, xyz_minmax, ouput: room_facet, room_vertices, room_norm_vectors
# 3. room_planes_generation, input: room_vertices, room_norm_vectors, output: room_plane_boundaries,room_plane_norm_vector
# 4. room_planes_selection, output: room_plane_boundaries_new,room_plane_norm_vector_new

def house_stl_reading(file):
    mesh1=mesh.Mesh.from_file(file)
    norm_vector=mesh1.normals
    for i in range(len(norm_vector)):
        n1=norm_vector[i,0]
        n2=norm_vector[i,1]
        n3=norm_vector[i,2]
        n1 = n1 / sqrt(n1 ** 2 + n2 ** 2 + n3 ** 2)
        n2 = n2 / sqrt(n1 ** 2 + n2 ** 2 + n3 ** 2)
        n3 = n3 / sqrt(n1 ** 2 + n2 ** 2 + n3 ** 2)
        norm_vector[i, 0] = n1
        norm_vector[i, 1] = n2
        norm_vector[i, 2] = n3
    points=mesh1.points
    vertices=np.zeros((3*len(points),3))
    for i in range(len(points)):
        vertices[3*i,0:3]=0.001*points[i,0:3]
        vertices[3*i+1,0:3]=0.001*points[i,3:6]
        vertices[3*i+2,0:3]=0.001*points[i,6:9]
    facet=np.arange(1,len(norm_vector)*3+1).reshape((len(norm_vector),3))
    vector=mesh1.vectors
    return facet, norm_vector, vertices, vector

def house_stl_matplot(vertices):
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)
    vector=np.array(vertices).reshape((len(vertices)/3,3,3))
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(vector))
    scale = vertices.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set(xlim=[0,5],ylim=[0,5],zlim=[0,3],title="3D MODEL OF HOUSE",
             xlabel="x-axis",ylabel="y-axis",zlabel="z-axis")
    pyplot.show()

def room_vertices_segment(house_facet,house_vertices,house_norm_vector):
    # the first part: house process
    house_facet_num=len(house_facet)
    house_facet_new=np.zeros((house_facet_num,3))
    house_vertices_new=np.zeros((3*house_facet_num,3))
    house_norm_vector_new=np.zeros((house_facet_num,3))
    for i in range(house_facet_num):
        house_facet_new[i,0:3]=house_facet[i,0:3]

        house_vertices_new[3*i:3*i+3,0]=house_vertices[3*i:3*i+3,0]
        house_vertices_new[3*i:3*i+3,1]=house_vertices[3*i:3*i+3,2]
        house_vertices_new[3*i:3*i+3,2]=house_vertices[3*i:3*i+3,1]

        house_norm_vector_new[i,0]=house_norm_vector[i,0]
        house_norm_vector_new[i,1]=house_norm_vector[i,2]
        house_norm_vector_new[i,2]=house_norm_vector[i,1]
    house_vertices_new=np.array(house_vertices_new)
    house_norm_vector_new=np.array(house_norm_vector_new)

    house_vertices_1=np.zeros((3*house_facet_num,3))
    house_norm_vector_1=np.zeros((house_facet_num,3))
    house_facet_1=house_facet
    for i in range(len(house_vertices_1)):
        house_vertices_1[i,0]=house_vertices_new[i,0]-house_vertices_new[0:len(house_vertices_1),0].min()
        house_vertices_1[i,1]=house_vertices_new[i,1]-house_vertices_new[0:len(house_vertices_1),1].min()
        house_vertices_1[i,2]=house_vertices_new[i,2]-house_vertices_new[0:len(house_vertices_1),2].min()
    house_norm_vector_1=house_norm_vector_new

    # the second part: room segment
    # xyz_minmax=np.array([-100,3200,-100,1600,-100,3000])*0.001 # these parameters are set as segment limitation for 0015.STL document
    xyz_minmax=np.array([345,10155,245,10255,195,3705])*0.001 # these parameters are set as segment limitation for 0016.STL document
    room_facet=[]
    room_norm_vector=[]
    room_vertices = []
    j=0
    for m in range(len(house_facet_1)):
        if np.all(house_vertices_1[3*m:3*m+3,0]>xyz_minmax[0]) and np.all(house_vertices_1[3*m:3*m+3,0]<xyz_minmax[1]):
            if np.all(house_vertices_1[3*m:3*m+3,1]>xyz_minmax[2]) and np.all(house_vertices_1[3*m:3*m+3,1]<xyz_minmax[3]):
                if np.all(house_vertices_1[3*m:3*m+3,2]>xyz_minmax[4]) and np.all(house_vertices_1[3*m:3*m+3,2]<xyz_minmax[5]):
                    room_facet=np.append(room_facet,[3*j+1,3*j+2,3*j+3],axis=0)
                    room_norm_vector=np.append(room_norm_vector,house_norm_vector_1[m, 0:3], axis=0)

                    house_vertices_array=np.array(house_vertices_1[3*m:3*m+3,0:3]).reshape(9)
                    room_vertices=np.append(room_vertices,house_vertices_array,axis=0)
                    j=j+1
    room_facet=np.array(room_facet).reshape((len(room_facet)/3,3))
    room_norm_vector=np.array(room_norm_vector).reshape((len(room_norm_vector)/3,3))
    room_vertices=np.array(room_vertices).reshape((len(room_vertices)/3,3))

    # the third part: room process
    room_vertices_1 = np.zeros((3*len(room_facet), 3))
    room_facet_1 = room_facet
    room_norm_vector_1 = room_norm_vector
    for i in range(len(room_vertices)):
        room_vertices_1[i, 0] = room_vertices[i, 0] - room_vertices[0:len(house_vertices_1), 0].min()
        room_vertices_1[i, 1] = room_vertices[i, 1] - room_vertices[0:len(house_vertices_1), 1].min()
        room_vertices_1[i, 2] = room_vertices[i, 2] - room_vertices[0:len(house_vertices_1), 2].min()
    return room_facet_1, room_norm_vector_1, room_vertices_1

def room_renovation_plane_vertices_selection(room_facet, room_norm_vector, room_vertices):
    # here we select room ceiling as our target surface
    vertices_flag1=np.ones(len(room_norm_vector))
    for i in range(len(room_norm_vector)):
        if abs(room_norm_vector[i,0])<=0.01 and abs(room_norm_vector[i,1])<=0.01 and abs(room_norm_vector[i,2])<=1.01:
            vertices_flag1[i]=0
    vertices_flag2=np.ones(len(room_norm_vector))
    for i in range(len(room_norm_vector)):
        if abs(room_vertices[3*i,2])>=1.0 and abs(room_vertices[3*i+1,2])>=1.0 and abs(room_vertices[3*i+2,2])>=1.0:
            vertices_flag2[i]=0
    k=0
    room_renovation_facet=[]
    room_renovation_norm_vector=[]
    room_renovation_vertices=[]
    for i in range(len(room_norm_vector)):
        if vertices_flag1[i]==0 and vertices_flag2[i]==0:
            room_renovation_facet=np.append(room_renovation_facet,[3*k+1,3*k+2,3*k+3],axis=0)
            room_renovation_norm_vector=np.append(room_renovation_norm_vector,room_norm_vector[i,0:3],axis=0)

            room_vertices_array=np.array(room_vertices[3*i:3*i+3,0:3]).reshape(9)
            room_renovation_vertices=np.append(room_renovation_vertices,room_vertices_array,axis=0)
            k=k+1
    room_renovation_facet=room_renovation_facet.reshape((len(room_renovation_facet)/3,3))
    room_renovation_norm_vector=room_renovation_norm_vector.reshape(len(room_renovation_norm_vector)/3,3)
    room_renovation_vertices=room_renovation_vertices.reshape(len(room_renovation_vertices)/3,3)
    return room_renovation_facet,room_renovation_norm_vector,room_renovation_vertices

def pointsort(points):
    p1=np.array(points[0:3])
    p2=np.array(points[3:6])
    if p1[0]>p2[0]:
        p1,p2=swap(p1,p2)
    elif p1[0]==p2[0]:
        if p1[1]>p2[1]:
            p1,p2=swap(p1,p2)
        elif p1[1]==p2[1]:
            if p1[2]>p2[2]:
                p1,p2=swap(p1,p2)
    points[0:3]=np.array(p1)
    points[3:6]=np.array(p2)
    return points
def swap(x,y):
    a=y
    b=x
    return a,b
def stl_room_planes_generation(room_vertices,room_norm_vector):
    index1=room_vertices.shape
    room_triangle=np.zeros((index1[0]/3,18))
    for i in range(index1[0]/3):
        p1=room_vertices[3*i,0:3]
        p2=room_vertices[3*i+1,0:3]
        p3=room_vertices[3*i+2,0:3]
        room_triangle[i,0:6]=pointsort(np.hstack((p1,p2)))
        room_triangle[i,6:12]=pointsort(np.hstack((p2,p3)))
        room_triangle[i,12:18]=pointsort(np.hstack((p1,p3)))
    triangle_num=index1[0]/3
    flag1=np.ones(triangle_num)
    room_triangle_cell={}
    room_triangle_cell.setdefault(0,{})[0]=room_triangle[0,0:18]
    room_triangle_norm_vector_cell={}
    room_triangle_norm_vector_cell.setdefault(0,{})[0]=room_norm_vector[0,0:3]
    # print(room_triangle_cell[0][0])
    # print(room_triangle_norm_vector_cell[0][0])
    flag1[0]=0
    time=1
    while(1):
        time+=1
        # print("time:",time)
        triangle_cell_num=len(room_triangle_cell)
        flag2=np.ones(triangle_cell_num)
        for j in range(triangle_cell_num):
            triangle_cell_triangle_num=len(room_triangle_cell[j])
            triangle_cell_triangle_num_before=triangle_cell_triangle_num
            for k in range(triangle_cell_triangle_num):
                for i in range(triangle_num):
                    triangle1=np.array(room_triangle[i,0:18])
                    line1=np.array(room_triangle_cell[j][k][0:6])
                    line2=np.array(room_triangle_cell[j][k][6:12])
                    line3=np.array(room_triangle_cell[j][k][12:18])
                    flag3=0
                    flag4=0
                    flag5=0
                    flag6=0
                    if (triangle1[0:6]==line1).all() or (triangle1[0:6]==line2).all() or (triangle1[0:6]==line3).all():
                        flag4=1
                    if (triangle1[6:12]==line1).all() or (triangle1[6:12]==line2).all() or (triangle1[6:12]==line3).all():
                        flag5=1
                    if (triangle1[12:18]==line1).all() or (triangle1[12:18]==line2).all() or (triangle1[12:18]==line3).all():
                        flag6=1
                    if flag4==1 or flag5==1 or flag6==1:
                        flag3=1
                    n=len(room_triangle_cell[j])
                    # print("n=:",n)
                    # print(room_triangle_cell)
                    for n in range(len(room_triangle_cell[j])):
                        # triangle2=np.array(room_triangle_cell[j][n][0:18])
                        triangle2 = np.array(room_triangle_cell[j][n])
                        if (triangle1==triangle2).all():
                            flag3=0
                    if room_norm_vector[i,0]!=room_triangle_norm_vector_cell[j][k][0] or room_norm_vector[i,1]!=room_triangle_norm_vector_cell[j][k][1] or room_norm_vector[i,2]!=room_triangle_norm_vector_cell[j][k][2]:
                        flag3=0
                    if flag3==1:
                        triangle_cell_triangle_num+=1
                        room_triangle_cell.setdefault(j, {})[triangle_cell_triangle_num-1] = room_triangle[i, 0:18]
                        room_triangle_norm_vector_cell.setdefault(j, {})[triangle_cell_triangle_num-1] = room_norm_vector[i, 0:3]
                        flag1[i]=0
            triangle_cell_triangle_num_after=len(room_triangle_cell[j])
            if triangle_cell_triangle_num_after!=triangle_cell_triangle_num_before:
                flag2[j]=0
        if (flag1==np.zeros(triangle_num)).all():
            break
        if (flag2==np.ones(triangle_cell_num)).all():
            triangle_cell_num+=1
            index=np.min(np.where(flag1==1)[0])
            room_triangle_cell.setdefault(triangle_cell_num-1, {})[0] = room_triangle[index, 0:18]
            room_triangle_norm_vector_cell.setdefault(triangle_cell_num-1, {})[0] = room_norm_vector[index, 0:3]
            flag1[index]=0
    # print(room_triangle_norm_vector_cell)
    room_triangle_edge_cell={}
    for i in range(len(room_triangle_cell)):
        for j in range(len(room_triangle_cell[i])):
            room_triangle_edge_cell.setdefault(i, {})[3 * j] = room_triangle_cell[i][j][0:6]
            room_triangle_edge_cell.setdefault(i, {})[3 * j+1] = room_triangle_cell[i][j][6:12]
            room_triangle_edge_cell.setdefault(i, {})[3 * j+2] = room_triangle_cell[i][j][12:18]
    room_plane_edge_cell={}
    room_plane_norm_vector={}
    for i in range(len(room_triangle_edge_cell)):
        m=0
        for j in range(len(room_triangle_edge_cell[i])):
            adjacent_num=0
            for k in range(len(room_triangle_edge_cell[i])):
                if j!=k:
                    if (np.array(room_triangle_edge_cell[i][j])==np.array(room_triangle_edge_cell[i][k])).all():
                        adjacent_num+=1
            if adjacent_num==0:
                room_plane_edge_cell.setdefault(i,{})[m]=room_triangle_edge_cell[i][j][0:6]
                m+=1
        room_plane_norm_vector.setdefault(i,{})[0]=room_triangle_norm_vector_cell[i][0][0:3]
    return room_plane_edge_cell, room_triangle_edge_cell, room_plane_norm_vector

def room_planes_matplot(room_plane_edge_cell):
    fig = pyplot.figure()
    ax1=Axes3D(fig)
    for i in range(len(room_plane_edge_cell)):
        for j in range(len(room_plane_edge_cell[i])):
            x1 = np.array([room_plane_edge_cell[i][j][0], room_plane_edge_cell[i][j][3]])
            y1 = np.array([room_plane_edge_cell[i][j][1], room_plane_edge_cell[i][j][4]])
            z1 = np.array([room_plane_edge_cell[i][j][2], room_plane_edge_cell[i][j][5]])
            ax1.plot3D(x1, y1, z1, 'gray')
    pyplot.show()

def boundary_cell_generation(plane_boundaries):
    boundary_num=len(plane_boundaries)
    flag1=np.ones(boundary_num)
    flag1[0]=0

    boundary_cell={}
    boundary_cell.setdefault(0,{})[0]=plane_boundaries[0][0:6]
    while(1):
        boundary_cell_num=len(boundary_cell)
        flag2=np.ones(boundary_cell_num)
        for j in range(boundary_cell_num):
            boundary_cell_boundary_num=len(boundary_cell[j])
            boundary_cell_boundary_num_before=boundary_cell_boundary_num
            for k in range(boundary_cell_boundary_num):
                for i in range(boundary_num):
                    p1=np.array(plane_boundaries[i][0:6])
                    p2=np.array(boundary_cell[j][k][0:3])
                    p3=np.array(boundary_cell[j][k][3:6])
                    flag3=0
                    flag4=0
                    flag5=0
                    flag6=0
                    flag7=0
                    if p1[0]==p2[0] and p1[1]==p2[1] and p1[2]==p2[2]:
                        flag4=1
                    if p1[3]==p3[0] and p1[4]==p3[1] and p1[5]==p3[2]:
                        flag5=1
                    if p1[0]==p3[0] and p1[1]==p3[1] and p1[2]==p3[2]:
                        flag6=1
                    if p1[3]==p2[0] and p1[4]==p2[1] and p1[5]==p2[2]:
                        flag7=1
                    if flag4==1 or flag5==1 or flag6==1 or flag7==1:
                        flag3=1
                    for n in range(len(boundary_cell[j])):
                        p4=np.array(boundary_cell[j][n][0:6])
                        if (p1==p4).all():
                            flag3=0
                    if flag3==1:
                        boundary_cell_boundary_num+=1
                        boundary_cell.setdefault(j,{})[boundary_cell_boundary_num-1]=plane_boundaries[i][0:6]
                        flag1[i]=0
            boundary_cell_boundary_num_after=len(boundary_cell[j])
            if boundary_cell_boundary_num_after!=boundary_cell_boundary_num_before:
                flag2[j]=0
        if (flag1==np.zeros(boundary_num)).all():
            break
        if (flag2==np.ones(boundary_cell_num)).all():
            boundary_cell_num+=1
            index=np.min(np.where(flag1==1))
            boundary_cell.setdefault(boundary_cell_num-1,{})[0]=plane_boundaries[index][0:6]
            flag1[index]=0
    return boundary_cell
def boundary_cell_sorting(boundary_cell):
    boundary_cell_sort={}
    for i in range(len(boundary_cell)):
        for j in range(len(boundary_cell[i])):
            array1=boundary_cell[i][j][0:6]
            array1=np.append(array1,0)
            boundary_cell_sort.setdefault(i,{})[j]=array1
    boundary_cell_range=np.zeros((len(boundary_cell),6))
    for i in range(len(boundary_cell)):
        x_array1 = np.zeros(len(boundary_cell[i]))
        x_array2 = np.zeros(len(boundary_cell[i]))
        y_array1 = np.zeros(len(boundary_cell[i]))
        y_array2 = np.zeros(len(boundary_cell[i]))
        z_array1 = np.zeros(len(boundary_cell[i]))
        z_array2 = np.zeros(len(boundary_cell[i]))
        for j in range(len(boundary_cell[i])):
            x_array1[j] = np.array(boundary_cell[i][j][0])
            x_array2[j] = np.array(boundary_cell[i][j][3])
            y_array1[j] = np.array(boundary_cell[i][j][1])
            y_array2[j] = np.array(boundary_cell[i][j][4])
            z_array1[j] = np.array(boundary_cell[i][j][2])
            z_array2[j] = np.array(boundary_cell[i][j][5])
        xmin=min(x_array1.min(),x_array2.min())
        xmax=max(x_array1.max(),x_array2.max())
        ymin=min(y_array1.min(),y_array2.min())
        ymax=max(y_array1.max(),y_array2.max())
        zmin=min(z_array1.min(),z_array2.min())
        zmax=max(z_array1.max(),z_array2.max())
        boundary_cell_range[i,0:6]=[xmin,xmax,ymin,ymax,zmin,zmax]
    xxmin=min(boundary_cell_range[0:len(boundary_cell_range),0])
    xxmax=max(boundary_cell_range[0:len(boundary_cell_range),1])
    yymin=min(boundary_cell_range[0:len(boundary_cell_range),2])
    yymax=max(boundary_cell_range[0:len(boundary_cell_range),3])
    zzmin=min(boundary_cell_range[0:len(boundary_cell_range),4])
    zzmax=max(boundary_cell_range[0:len(boundary_cell_range),5])
    xyz_range=np.array([xxmin,xxmax,yymin,yymax,zzmin,zzmax])
    for i in range(len(boundary_cell)):
        if (boundary_cell_range[i,0:6]==xyz_range).all():
            for j in range(len(boundary_cell[i])):
                # array2 = boundary_cell[i][j][0:6]
                # array2 = np.append(array2,1)
                # boundary_cell_sort.setdefault(i, {})[j] = array2
                boundary_cell_sort[i][j][6]=1
    return boundary_cell_sort
def room_planes_paths_generation(room_plane_edge_cell,room_plane_norm_vector):
    # pre-work for parameters setting
    interval1=200*0.001
    interval2=50*0.001
    # the first and second step: add the function of boundary_cell_generation and boundary_cell_sorting
    room_planes_boundary_cell={}
    room_planes_boundary_cell_sorted={}
    for i in range(len(room_plane_edge_cell)):
        plane_boundaries=room_plane_edge_cell[i]
        boundary_cell=boundary_cell_generation(plane_boundaries)
        room_planes_boundary_cell[i] = boundary_cell
        boundary_cell_sorted=boundary_cell_sorting(boundary_cell)
        room_planes_boundary_cell_sorted[i]=boundary_cell_sorted
    # the third step: obtain the outer boundary of each plane
    room_plane_boundary=room_planes_boundary_cell_sorted

    # print(len(room_plane_boundary[0]))
    # print(len(room_plane_boundary[0][0]))
    # print(len(room_plane_boundary[0][0][0]))
    # print(room_plane_boundary[0][0][0])

    room_plane_outer_boundary={}
    for i in range(len(room_plane_boundary)):
        m=0
        for j in range(len(room_plane_boundary[i])):
            if room_plane_boundary[i][j][0][6]==1:
                for k in range(len(room_plane_boundary[i][j])):
                    room_plane_outer_boundary.setdefault(i,{})[m]=room_plane_boundary[i][j][k][0:6]
                    m=m+1
    # print(len(room_plane_outer_boundary))
    # print(len(room_plane_outer_boundary[0]))
    # print(len(room_plane_outer_boundary[0][0]))

    room_plane_outer_boundary_rep={}
    for i in range(len(room_plane_outer_boundary)):
        m=0
        for j in range(len(room_plane_outer_boundary[i])):
            room_plane_outer_boundary_rep.setdefault(i,{})[m]=room_plane_outer_boundary[i][j][0:3]
            m=m+1
            room_plane_outer_boundary_rep.setdefault(i,{})[m]=room_plane_outer_boundary[i][j][3:6]
            m=m+1
    # print(len(room_plane_outer_boundary_rep))
    # print(len(room_plane_outer_boundary_rep[0]))
    # print(len(room_plane_outer_boundary_rep[0][0]))

    room_plane_outer_boundary_points = {}
    # for i in range(1):
    for i in range(len(room_plane_outer_boundary_rep)):
        point_rep_mat = []
        for j in range(len(room_plane_outer_boundary_rep[i])):
            array1=tuple(room_plane_outer_boundary_rep[i][j])
            point_rep_mat=np.append(point_rep_mat,array1,axis=0)
        point_rep_mat=point_rep_mat.reshape((len(point_rep_mat)/3,3))
        room_plane_outer_boundary_points.setdefault(i, {})[0]= np.unique(point_rep_mat, axis=0)
    # print(room_plane_outer_boundary_points)
    # print(len(room_plane_outer_boundary_points[0][0][0]))
    # print(room_plane_outer_boundary_points[1][0][1])

    # the fourth step: obtain intersection points inside one plane
    room_plane_point=np.zeros((len(room_plane_norm_vector),3))
    room_plane_parameters=np.zeros((len(room_plane_norm_vector),4))
    for i in range(len(room_plane_norm_vector)):
        room_plane_point[i,0:3]=room_plane_boundary[i][0][0][0:3]
        room_plane_parameters[i,0:3]=room_plane_norm_vector[i][0][0:3]
        room_plane_parameters[i,3]=-np.dot(room_plane_norm_vector[i][0][0:3],room_plane_point[i,0:3].T)
    intersect_plane_norm_vector1=np.zeros((len(room_plane_norm_vector),3))
    intersect_plane_norm_vector2=np.zeros((len(room_plane_norm_vector),3))
    for i in range(len(room_plane_norm_vector)):
        sin_theta=-room_plane_parameters[i,0]
        cos_theta=sqrt(1-sin_theta**2)
        if cos_theta!=0:
            sin_beta=room_plane_parameters[i,1]/cos_theta
            cos_beta=room_plane_parameters[i,2]/cos_theta
        else:
            sin_beta=0
            cos_beta=1
        intersect_plane_norm_vector1[i,0:3]=[cos_theta,sin_theta*sin_beta,sin_theta*cos_beta]
        intersect_plane_norm_vector2[i,0:3]=[0,cos_beta,-sin_beta]
    # print(room_plane_parameters)
    # print(intersect_plane_norm_vector1)
    # print(intersect_plane_norm_vector2)
    intersect_plane1_d_candidate={}
    intersect_plane2_d_candidate={}
    for i in range(len(room_plane_outer_boundary_points)):
        for j in range(len(room_plane_outer_boundary_points[i][0])):
            intersect_plane1_d_candidate.setdefault(i,{})[j]=-np.dot(intersect_plane_norm_vector1[i,0:3],room_plane_outer_boundary_points[i][0][j,0:3].T)
            intersect_plane2_d_candidate.setdefault(i,{})[j]=-np.dot(intersect_plane_norm_vector2[i,0:3],room_plane_outer_boundary_points[i][0][j,0:3].T)
            # print(intersect_plane_norm_vector2[i,0:3])
            # print(room_plane_outer_boundary_points[i][0][j,0:3].T)
    # print(intersect_plane1_d_candidate)
    # print(intersect_plane2_d_candidate)

    intersect_plane1_dmin_max=np.zeros((len(room_plane_outer_boundary_points),2))
    intersect_plane2_dmin_max=np.zeros((len(room_plane_outer_boundary_points),2))
    for i in range(len(intersect_plane1_d_candidate)):
        d_candidate_array_plane1=[]
        d_candidate_array_plane2=[]
        for j in range(len(intersect_plane1_d_candidate[i])):
            d_candidate_array_plane1=np.append(d_candidate_array_plane1,intersect_plane1_d_candidate[i][j])
            d_candidate_array_plane2=np.append(d_candidate_array_plane2,intersect_plane2_d_candidate[i][j])
        intersect_plane1_dmin_max[i,0]=d_candidate_array_plane1.min()
        intersect_plane1_dmin_max[i,1]=d_candidate_array_plane1.max()
        intersect_plane2_dmin_max[i,0]=d_candidate_array_plane2.min()
        intersect_plane2_dmin_max[i,1]=d_candidate_array_plane2.max()
    # print(intersect_plane1_dmin_max)
    # print(intersect_plane2_dmin_max)
    room_plane_interval=[]
    for i in range(len(room_plane_outer_boundary_points)):
        l1=abs(intersect_plane1_dmin_max[i,1]-intersect_plane1_dmin_max[i,0])
        l2=abs(intersect_plane2_dmin_max[i,1]-intersect_plane2_dmin_max[i,0])
        if l1>=l2:
            room_plane_interval=np.append(room_plane_interval,interval1)
            room_plane_interval=np.append(room_plane_interval,interval2)
        else:
            room_plane_interval=np.append(room_plane_interval,interval2)
            room_plane_interval=np.append(room_plane_interval,interval1)
    room_plane_interval=np.array(room_plane_interval).reshape((len(room_plane_interval)/2,2))
    intersect_plane1_d={}
    for i in range(len(room_plane_outer_boundary_points)):
        dmin=intersect_plane1_dmin_max[i,0]
        dmax=intersect_plane1_dmin_max[i,1]
        interval=room_plane_interval[i,0]
        plane1_num=int(floor(abs(dmax-dmin)/interval))
        for j in range(plane1_num-1):
            intersect_plane1_d.setdefault(i,{})[j]=dmin+(j+1)*interval
            pt=dmin+j*interval
            if pt>dmax:
                intersect_plane1_d.setdefault(i,{})[j]=dmax
    intersect_plane2_d={}
    for i in range(len(room_plane_outer_boundary_points)):
        dmin=intersect_plane2_dmin_max[i,0]
        dmax=intersect_plane2_dmin_max[i,1]
        interval=room_plane_interval[i,1]
        plane2_num=int(floor(abs(dmax-dmin)/interval))
        for j in range(plane2_num-1):
            intersect_plane2_d.setdefault(i,{})[j]=dmin+(j+1)*interval
            pt=dmin+j*interval
            if pt>dmax:
                intersect_plane2_d.setdefault(i,{})[j]=dmax
    # print(intersect_plane2_dmin_max)
    # print(intersect_plane2_d)
    room_plane_intersect_point={}
    for i in range(len(room_plane_outer_boundary_points)):
        m=0
        for j in range(len(intersect_plane1_d[i])):
            mat1=np.zeros((3,3))
            mat2=np.zeros((3,1))
            for k in range(len(intersect_plane2_d[i])):
                mat1[0,0:3]=room_plane_parameters[i,0:3]
                mat1[1,0:3]=intersect_plane_norm_vector1[i,0:3]
                mat1[2,0:3]=intersect_plane_norm_vector2[i,0:3]
                mat1=np.matrix(mat1)
                mat2[0,0]=-room_plane_parameters[i,3]
                mat2[1,0]=-intersect_plane1_d[i][j]
                mat2[2,0]=-intersect_plane2_d[i][k]
                mat2=np.matrix(mat2)
                mat3=np.dot(mat1.I,mat2)
                room_plane_intersect_point.setdefault(i,{})[m]=[mat3[0,0],mat3[1,0],mat3[2,0]]
                m=m+1
                # room_plane_intersect_point.setdefault(i,{})[j,k]=[0,0,0]
        # print(len(intersect_plane1_d[i]))
        # print(len(intersect_plane2_d[i]))
        # print(len(intersect_plane1_d[i])*len(intersect_plane2_d[i]))
    # print(len(room_plane_intersect_point[0]))

    intersect_plane1_d_min_max=np.zeros((len(intersect_plane1_d),2))
    for i in range(len(intersect_plane1_d)):
        array1=[]
        for j in range(len(intersect_plane1_d[i])):
            array1=np.append(array1,intersect_plane1_d[i][j])
        intersect_plane1_d_min_max[i,0]=array1.min()
        intersect_plane1_d_min_max[i,1]=array1.max()
    # print(intersect_plane1_d_min_max)

    intersect_plane2_d_min_max = np.zeros((len(intersect_plane2_d), 2))
    for i in range(len(intersect_plane2_d)):
        array1 = []
        for j in range(len(intersect_plane2_d[i])):
            array1 = np.append(array1, intersect_plane2_d[i][j])
        intersect_plane2_d_min_max[i, 0] = array1.min()
        intersect_plane2_d_min_max[i, 1] = array1.max()
    # print(intersect_plane2_d_min_max)

    # obtatin effective intersection points inside one plane
    room_plane_effective_intersect_point=room_plane_intersect_point
    print(len(intersect_plane1_d[0]))
    print(len(intersect_plane2_d[0]))
    return room_plane_effective_intersect_point

def room_ceiling_waypoint_generation(room_plane_effective_intersect_point):
    interval=0.050
    for i in range(len(room_plane_effective_intersect_point)):
        array1=np.zeros(len(room_plane_effective_intersect_point[i]))
        array2=np.zeros(len(room_plane_effective_intersect_point[i]))
        array3=np.zeros(len(room_plane_effective_intersect_point[i]))
        for j in range(len(room_plane_effective_intersect_point[i])):
            array1[j]=room_plane_effective_intersect_point[i][j][0]
            array2[j]=room_plane_effective_intersect_point[i][j][1]
            array3[j]=room_plane_effective_intersect_point[i][j][2]
        xmax=array1.max()
        ymax=array2.max()
        z_equal=(array2.min()+array2.max())/2
        room_ceiling_waypoints_mat=np.ones((int(floor(xmax/interval)),int(floor(ymax/interval))))
        for k in range(len(room_plane_effective_intersect_point[i])):
            m=int(floor(room_plane_effective_intersect_point[i][k][0]/interval))-1
            n=int(floor(room_plane_effective_intersect_point[i][k][1]/interval))-1
            room_ceiling_waypoints_mat[m,n]=0.0
    return room_ceiling_waypoints_mat
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
def visualization_cube(num, position, color, scale):
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.ns = 'cube'
    marker.id = num
    marker.lifetime = rospy.Duration()

    marker.color.r = color.r
    marker.color.g = color.g
    marker.color.b = color.b
    marker.color.a = color.a

    marker.pose.position.x = position.x
    marker.pose.position.y = position.y
    marker.pose.position.z = position.z
    return marker
def rviz_visualization_stl(str1):
    str='package://aubo_demo/cpp_planner/stl_document1/'
    str=str+str1
    color=ColorRGBA()
    color.r=1.0
    color.g=0.0
    color.b=0.0
    color.a=1.0
    position = Point()
    position.x=0.0
    position.y=0.0
    position.z=0.0

    marker=Marker()
    marker.header.frame_id="/world"
    marker.type=Marker.MESH_RESOURCE
    #marker.mesh_resource = "package://aubo_demo/stl_test/stl_document/0015.STL"
    marker.mesh_resource=str

    marker.action=Marker.ADD
    marker.pose.orientation.w=1.0
    marker.scale.x=0.001
    marker.scale.y=0.001
    marker.scale.z=0.001
    marker.ns='arrow'
    marker.id=1
    marker.lifetime = rospy.Duration()

    marker.color.r=color.r
    marker.color.g=color.g
    marker.color.b=color.b
    marker.color.a=color.a

    marker.pose.position.x=position.x
    marker.pose.position.y=position.y
    marker.pose.position.z=position.z

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('markers')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()

def room_vertices_transformation(room_vertices):
    room_stl = mesh.Mesh(np.zeros(len(room_vertices)/3, dtype=mesh.Mesh.dtype))
    for i in range(len(room_vertices)/3):
        room_stl.points[i, 0:3] = room_vertices[3*i, 0:3]
        room_stl.points[i, 3:6] = room_vertices[3*i+1, 0:3]
        room_stl.points[i, 6:9] = room_vertices[3*i+2, 0:3]
    room_stl.save("stl_document/room_0015.STL")
def room_surface_division_planes(room_plane_edge_cell,room_plane_norm_vector):
    # pre-work for parameters setting

    # the first and second step: add the function of boundary_cell_generation and boundary_cell_sorting
    room_planes_boundary_cell={}
    room_planes_boundary_cell_sorted={}
    for i in range(len(room_plane_edge_cell)):
        plane_boundaries=room_plane_edge_cell[i]
        boundary_cell=boundary_cell_generation(plane_boundaries)
        room_planes_boundary_cell[i] = boundary_cell
        boundary_cell_sorted=boundary_cell_sorting(boundary_cell)
        room_planes_boundary_cell_sorted[i]=boundary_cell_sorted

    # the third step: obtain the outer boundary of each plane
    room_plane_boundary=room_planes_boundary_cell_sorted
    room_plane_outer_boundary={}
    for i in range(len(room_plane_boundary)):
        m=0
        for j in range(len(room_plane_boundary[i])):
            if room_plane_boundary[i][j][0][6]==1:
                for k in range(len(room_plane_boundary[i][j])):
                    room_plane_outer_boundary.setdefault(i,{})[m]=room_plane_boundary[i][j][k][0:6]
                    m=m+1

    room_plane_outer_boundary_rep={}
    for i in range(len(room_plane_outer_boundary)):
        m=0
        for j in range(len(room_plane_outer_boundary[i])):
            room_plane_outer_boundary_rep.setdefault(i,{})[m]=room_plane_outer_boundary[i][j][0:3]
            m=m+1
            room_plane_outer_boundary_rep.setdefault(i,{})[m]=room_plane_outer_boundary[i][j][3:6]
            m=m+1

    room_plane_outer_boundary_points = {}
    for i in range(len(room_plane_outer_boundary_rep)):
        point_rep_mat = []
        for j in range(len(room_plane_outer_boundary_rep[i])):
            array1=tuple(room_plane_outer_boundary_rep[i][j])
            point_rep_mat=np.append(point_rep_mat,array1,axis=0)
        point_rep_mat=point_rep_mat.reshape((8,3))
        room_plane_outer_boundary_points.setdefault(i, {})[0]= np.unique(point_rep_mat, axis=0)

    # the fourth step: obtain intersection points inside one plane
    room_plane_point=np.zeros((len(room_plane_norm_vector),3))
    room_plane_parameters=np.zeros((len(room_plane_norm_vector),4))
    for i in range(len(room_plane_norm_vector)):
        room_plane_point[i,0:3]=room_plane_boundary[i][0][0][0:3]
        room_plane_parameters[i,0:3]=room_plane_norm_vector[i][0][0:3]
        room_plane_parameters[i,3]=-np.dot(room_plane_norm_vector[i][0][0:3],room_plane_point[i,0:3].T)
    intersect_plane_norm_vector1=np.zeros((len(room_plane_norm_vector),3))
    intersect_plane_norm_vector2=np.zeros((len(room_plane_norm_vector),3))
    for i in range(len(room_plane_norm_vector)):
        sin_theta=-room_plane_parameters[i,0]
        cos_theta=sqrt(1-sin_theta**2)
        if cos_theta!=0:
            sin_beta=room_plane_parameters[i,1]/cos_theta
            cos_beta=room_plane_parameters[i,2]/cos_theta
        else:
            sin_beta=0
            cos_beta=1
        intersect_plane_norm_vector1[i,0:3]=[cos_theta,sin_theta*sin_beta,sin_theta*cos_beta]
        intersect_plane_norm_vector2[i,0:3]=[0,cos_beta,-sin_beta]

    intersect_plane1_d_candidate={}
    intersect_plane2_d_candidate={}
    for i in range(len(room_plane_outer_boundary_points)):
        for j in range(len(room_plane_outer_boundary_points[i][0])):
            intersect_plane1_d_candidate.setdefault(i,{})[j]=-np.dot(intersect_plane_norm_vector1[i,0:3],room_plane_outer_boundary_points[i][0][j,0:3].T)
            intersect_plane2_d_candidate.setdefault(i,{})[j]=-np.dot(intersect_plane_norm_vector2[i,0:3],room_plane_outer_boundary_points[i][0][j,0:3].T)

    intersect_plane1_dmin_max=np.zeros((len(room_plane_outer_boundary_points),2))
    intersect_plane2_dmin_max=np.zeros((len(room_plane_outer_boundary_points),2))
    for i in range(len(intersect_plane1_d_candidate)):
        d_candidate_array_plane1=[]
        d_candidate_array_plane2=[]
        for j in range(len(intersect_plane1_d_candidate[i])):
            d_candidate_array_plane1=np.append(d_candidate_array_plane1,intersect_plane1_d_candidate[i][j])
            d_candidate_array_plane2=np.append(d_candidate_array_plane2,intersect_plane2_d_candidate[i][j])
        intersect_plane1_dmin_max[i,0]=d_candidate_array_plane1.min()
        intersect_plane1_dmin_max[i,1]=d_candidate_array_plane1.max()
        intersect_plane2_dmin_max[i,0]=d_candidate_array_plane2.min()
        intersect_plane2_dmin_max[i,1]=d_candidate_array_plane2.max()

    # the fifth step: here the intersection planes for division of different cells are computed as follows:
    limit_interval1=800*0.001
    limit_interval2=800*0.001

    room_plane_limit_interval=[]
    for i in range(len(room_plane_outer_boundary_points)):
        l1=abs(intersect_plane1_dmin_max[i,1]-intersect_plane1_dmin_max[i,0])
        l2=abs(intersect_plane2_dmin_max[i,1]-intersect_plane2_dmin_max[i,0])
        if l1>=l2:
            room_plane_limit_interval=np.append(room_plane_limit_interval,limit_interval1)
            room_plane_limit_interval=np.append(room_plane_limit_interval,limit_interval2)
        else:
            room_plane_limit_interval=np.append(room_plane_limit_interval,limit_interval2)
            room_plane_limit_interval=np.append(room_plane_limit_interval,limit_interval1)
    room_plane_limit_interval=np.array(room_plane_limit_interval).reshape((len(room_plane_limit_interval)/2,2))

    intersect_limit_plane1_d={}
    intersect_limit_plane2_d={}
    for i in range(len(intersect_plane1_dmin_max)):
        dmin=intersect_plane1_dmin_max[i,0]
        dmax=intersect_plane1_dmin_max[i,1]
        interval=room_plane_limit_interval[i,0]
        plane1_num=int(floor(abs(dmax-dmin)/interval))
        for j in range(plane1_num+2):
            intersect_limit_plane1_d.setdefault(i,{})[j]=dmin+j*interval
        dmin = intersect_plane2_dmin_max[i, 0]
        dmax = intersect_plane2_dmin_max[i, 1]
        interval = room_plane_limit_interval[i, 1]
        plane2_num = int(floor(abs(dmax - dmin) / interval))
        for j in range(plane2_num + 2):
            intersect_limit_plane2_d.setdefault(i,{})[j]=dmin+j*interval

    # the sixth step: here the intersection planes are computed as follows:
    interval1 = 200 * 0.001
    interval2 = 50 * 0.001
    room_plane_interval = []
    for i in range(len(room_plane_outer_boundary_points)):
        l1 = abs(intersect_plane1_dmin_max[i, 1] - intersect_plane1_dmin_max[i, 0])
        l2 = abs(intersect_plane2_dmin_max[i, 1] - intersect_plane2_dmin_max[i, 0])
        if l1 >= l2:
            room_plane_interval = np.append(room_plane_interval, interval1)
            room_plane_interval = np.append(room_plane_interval, interval2)
        else:
            room_plane_interval = np.append(room_plane_interval, interval2)
            room_plane_interval = np.append(room_plane_interval, interval1)
    room_plane_interval = np.array(room_plane_interval).reshape((len(room_plane_interval) / 2, 2))
    intersect_plane1_d = {}
    intersect_plane2_d = {}
    for i in range(len(intersect_plane1_dmin_max)):
        dmin = intersect_plane1_dmin_max[i, 0]
        dmax = intersect_plane1_dmin_max[i, 1]
        interval = room_plane_interval[i, 0]
        plane1_num = int(floor(abs(dmax - dmin) / interval))
        for j in range(plane1_num - 1):
            intersect_plane1_d.setdefault(i, {})[j] = dmin + (j + 1) * interval
            pt = dmin + j * interval
            if pt > dmax:
                intersect_plane1_d.setdefault(i, {})[j] = dmax
        dmin = intersect_plane2_dmin_max[i, 0]
        dmax = intersect_plane2_dmin_max[i, 1]
        interval = room_plane_interval[i, 1]
        plane2_num = int(floor(abs(dmax - dmin) / interval))
        for j in range(plane2_num - 1):
            intersect_plane2_d.setdefault(i, {})[j] = dmin + (j + 1) * interval
            pt = dmin + j * interval
            if pt > dmax:
                intersect_plane2_d.setdefault(i, {})[j] = dmax

    # the seventh step: computation of  room_plane_intersect_points_cell
    # input: intersect_plane1_d, intersect_plane2_d
    # input: intersect_limit_plane1_d, intersect_limit_plane2_d
    # output: intersect_plane1_d_cell
    # output: intersect_plane2_d_cell
    # output: room_plane_intersect_points_cell
    intersect_plane1_d_cell={}
    intersect_plane2_d_cell={}
    print(len(intersect_limit_plane1_d[0])-1)
    print(len(intersect_limit_plane2_d[0])-1)
    print(len(intersect_plane1_d[0])-1)
    print(len(intersect_plane2_d[0])-1)
    for i in range(len(intersect_limit_plane1_d)):
        # temp_dic_i={}
        for j in range(len(intersect_limit_plane1_d[i])-1):
            temp_dic_j={}
            for k in range(len(intersect_limit_plane2_d[i])-1):
                num = 0
                temp_m_k = {}
                tempdict = {}
                for m in range(len(intersect_plane1_d[i])-1):
                    for n in range(len(intersect_plane2_d[i])-1):
                        if intersect_limit_plane1_d[i][j]<=intersect_plane1_d[i][m] and intersect_plane1_d[i][m]<intersect_limit_plane1_d[i][j+1]:
                            if intersect_limit_plane2_d[i][k]<=intersect_plane2_d[i][n] and intersect_plane2_d[i][n]<intersect_limit_plane2_d[i][k+1]:

                                tempdict.update({num:intersect_plane1_d[i][m]})
                                temp_m_k.update({k:tempdict})
                                temp_dic_j.update({j:temp_m_k})
                                intersect_plane1_d_cell.update({i:temp_dic_j})

                                # intersect_plane1_d_cell.update({i:temp_dic_i})
                                # intersect_plane1_d_cell.setdefault(i,{})[j][k][num]=intersect_plane1_d[i][m]
                                # intersect_plane2_d_cell.setdefault(i,{})[j][k][num]=intersect_plane2_d[i][n]
                                num=num+1
    print(intersect_plane1_d_cell)
    # print(len(intersect_plane1_d_cell[0]))
    # intersect_plane1_d_cell[plane_num][cell_num_h][cell_num_v][point_num]=value
    # for i in range(len(intersect_plane1_d_cell)):
    #     for j in range(len()):
    # room_plane_intersect_point = {}
    # for i in range(len(room_plane_outer_boundary_points)):
    #     m = 0
    #     for j in range(len(intersect_plane1_d[i])):
    #         mat1 = np.zeros((3, 3))
    #         mat2 = np.zeros((3, 1))
    #         for k in range(len(intersect_plane2_d[i])):
    #             mat1[0, 0:3] = room_plane_parameters[i, 0:3]
    #             mat1[1, 0:3] = intersect_plane_norm_vector1[i, 0:3]
    #             mat1[2, 0:3] = intersect_plane_norm_vector2[i, 0:3]
    #             mat1 = np.matrix(mat1)
    #             mat2[0, 0] = -room_plane_parameters[i, 3]
    #             mat2[1, 0] = -intersect_plane1_d[i][j]
    #             mat2[2, 0] = -intersect_plane2_d[i][k]
    #             mat2 = np.matrix(mat2)
    #             mat3 = np.dot(mat1.I, mat2)
    #             room_plane_intersect_point.setdefault(i, {})[m] = [mat3[0, 0], mat3[1, 0], mat3[2, 0]]
    #             m = m + 1


if __name__ == "__main__":
    rviz_visualization_stl('0015.STL')
    # # for Ros command
    # house_facet,house_norm_vector,house_vertices,house_vector=house_stl_reading("stl_document1/0016.STL")
    # # for Python command
    # # house_facet, house_norm_vector, house_vertices, house_vector = house_stl_reading("0016.STL")
    #
    # room_facet_1,room_norm_vector_1,room_vertices_1=room_vertices_segment(house_facet,house_vertices,house_norm_vector)
    # room_renovation_facet,room_renovation_norm_vector,room_renovation_vertices=room_renovation_plane_vertices_selection(room_facet_1,room_norm_vector_1,room_vertices_1)
    # room_plane_edge_cell,room_triangle_edge_cell,room_plane_norm_vector=stl_room_planes_generation(room_renovation_vertices,room_renovation_norm_vector)
    #
    # room_plane_intersect_point=room_planes_paths_generation(room_plane_edge_cell, room_plane_norm_vector)
    # room_ceiling_waypoints_mat=room_ceiling_waypoint_generation(room_plane_intersect_point)
    #
    # mat_path="/home/zy/catkin_ws/src/aubo_robot/aubo_demo/cpp_planner/experiment_1/room_ceiling_waypoints_mat.mat"
    # io.savemat(mat_path,{"room_ceiling_waypoints_mat":room_ceiling_waypoints_mat})

    # # below code is applied for ROS RVIZ visualization
    # marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    # rospy.init_node('markers')
    # rate = rospy.Rate(10)
    # marker = Marker()
    # markerarray = MarkerArray()
    # position = Point()
    # color = ColorRGBA()
    # color.r = 1.0
    # color.g = 0.0
    # color.b = 0.0
    # color.a = 1.0
    # scale = [0.04, 0.04, 0.04]
    # num = 1
    # for i in range(len(room_ceiling_waypoints_mat)):
    #     for j in range(len(room_ceiling_waypoints_mat[i])):
    #         data = room_ceiling_waypoints_mat[i,j]
    #         position.x = (i+1)*0.050
    #         position.y = (j+1)*0.050
    #         position.z = 5.0
    #         color=compute_rgb(data)
    #         marker = visualization_cube(num, position, color, scale)
    #         num = num + 1
    #         markerarray.markers.append(marker)
    # while not rospy.is_shutdown():
    #     marker_pub.publish(markerarray)
    #     rate.sleep()





