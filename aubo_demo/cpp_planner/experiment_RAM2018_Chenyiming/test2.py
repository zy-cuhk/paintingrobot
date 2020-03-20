#!/usr/bin/env python
from math import *
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

def stl_matplot1(vertices):
    # Create a new plot
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)
    # Load the STL files and add the vectors to the plot
    new_vertices=np.array(vertices).reshape((len(vertices)/3,3,3))
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(new_vertices))
    # Auto scale to the mesh size
    scale = vertices.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set(xlim=[0,5],ylim=[0,5],zlim=[0,3],title="3D MODEL OF HOUSE",
             xlabel="x-axis",ylabel="y-axis",zlabel="z-axis")
    # Show the plot to the screen
    pyplot.show()
def stl_matplot2(file):
    mesh1 = mesh.Mesh.from_file(file)
    # Create a new plot
    figure = pyplot.figure()
    axes = mplot3d.Axes3D(figure)
    # Load the STL files and add the vectors to the plot
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh1.vectors))
    # Auto scale to the mesh size
    scale = mesh1.points.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)
    # Show the plot to the screen
    pyplot.show()

def stl_rotation(vertices):
    rotation_vertices = np.zeros((len(vertices), 3))
    for i in range(len(vertices)):
        for j in range(3):
            rotation_vertices[3 * i - 3:3 * i, 0] = vertices[3 * i - 3:3 * i, 0]
            rotation_vertices[3 * i - 3:3 * i, 1] = vertices[3 * i - 3:3 * i, 2]
            rotation_vertices[3 * i - 3:3 * i, 2] = vertices[3 * i - 3:3 * i, 1]
    new_stl = mesh.Mesh(np.zeros(len(rotation_vertices) / 3, dtype=mesh.Mesh.dtype))
    for i in range(len(rotation_vertices) / 3):
        new_stl.points[i, 0:3] = rotation_vertices[3 * i - 3, 0:3]
        new_stl.points[i, 3:6] = rotation_vertices[3 * i - 1, 0:3]
        new_stl.points[i, 6:9] = rotation_vertices[3 * i - 2, 0:3]
    new_stl.save("new_0015.STL")
    return rotation_vertices

def stl_room_segment(xyz_minmax, vertices):
    j = 1
    array1 = []
    for i in range(len(vertices) / 3):
        if vertices[3 * i - 3:3 * i, 0] > xyz_minmax[0] and vertices[3 * i - 3:3 * i, 0] < xyz_minmax[1]:
            if vertices[3 * i - 3:3 * i, 1] > xyz_minmax[2] and vertices[3 * i - 3:3 * i, 1] < xyz_minmax[3]:
                if vertices[3 * i - 3:3 * i, 2] > xyz_minmax[4] and vertices[3 * i - 3:3 * i, 2] < xyz_minmax[5]:
                    if j == 1:
                        array1 = np.append(array1, vertices[3 * i - 3:3 * i, 0:3], axis=0)
                    print(array1)
                    j = j + 1

# stl_room_surface_generation
