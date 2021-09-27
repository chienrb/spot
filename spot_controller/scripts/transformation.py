#!/usr/bin/env python
# author: lnotspot
import numpy as np
from math import sin, cos

# Functions for computing 3D rotation, transformation, and homogenenous transformation matrices

def rotx(alpha):
    # Create a 3x3 rotation matrix about the x axis
    rx = np.array([[1,          0,           0 ],
                   [0, cos(alpha), -sin(alpha) ],
                   [0, sin(alpha),  cos(alpha) ]])

    return rx


def roty(beta):
    # Create a 3x3 rotation matrix about the y axis
    ry = np.array([[ cos(beta), 0, sin(beta)],
                   [ 0        , 1,         0],
                   [-sin(beta), 0, cos(beta)]])

    return ry


def rotz(gamma):
    # Create a 3x3 rotation matrix about the z axis
    rz = np.array([[cos(gamma), -sin(gamma),  0],
                   [sin(gamma),  cos(gamma),  0],
                   [         0,           0,  1]])

    return rz

def homo_transxyz(dx, dy, dz):
    # Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)
    return np.block([ [np.eye(3,3) , np.array([[dx],[dy],[dz]]) ] , [np.array([0,0,0,1])] ])

def homo_rotxyz(alpha, beta, gamma):
    rotxyz = np.matmul(np.matmul(rotx(alpha), roty(beta)), rotz(gamma))
    return np.block([ [rotxyz , np.array([[0],[0],[0]])] , [np.array([0,0,0,1])] ])

def homo_transform(alpha,beta,gamma,dx,dy,dz):
    # Create a homogeneous 4x4 transformation matrix
    return np.matmul(homo_transxyz(dx,dy,dz), homo_rotxyz(alpha, beta, gamma))

def homo_inv(ht):
#    Return the inverse of a homogeneous transformation matrix.
#
#                 -------------------------
#                 |           |           |
#    inverse   =  |    R^T    |  -R^T * d |
#                 |___________|___________|
#                 | 0   0   0 |     1     |
#                 -------------------------

    temp_rot = ht[0:3,0:3].transpose()

    # Get the linear transformation portion of the transform, and multiply elements by -1
    temp_vec = -1*ht[0:3,3]

    # Block the inverted rotation matrix back to a 4x4 homogeneous transform matrix
    temp_rot_ht = np.block([ [temp_rot        , np.zeros((3,1))],
                             [np.zeros((1,3)) ,       np.eye(1)]  ])

    # Create a linear translation homogeneous transformation matrix
    temp_vec_ht = np.eye(4)
    temp_vec_ht[0:3,3] = temp_vec

    # Return the matrix product
    # return temp_rot_ht @ temp_vec_ht
    return np.matmul(temp_rot_ht, temp_vec_ht)
