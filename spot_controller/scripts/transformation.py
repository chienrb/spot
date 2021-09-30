#!/usr/bin/env python
# author: lnotspot
import numpy as np
from math import sin, cos

# Functions for computing 3D rotation, transformation, and homogenenous transformation matrices

def rotx(alpha):
    rx = np.array([[1,          0,           0 ],
                   [0, cos(alpha), -sin(alpha) ],
                   [0, sin(alpha),  cos(alpha) ]])

    return rx


def roty(beta):
    ry = np.array([[ cos(beta), 0, sin(beta)],
                   [ 0        , 1,         0],
                   [-sin(beta), 0, cos(beta)]])

    return ry


def rotz(gamma):
    rz = np.array([[cos(gamma), -sin(gamma),  0],
                   [sin(gamma),  cos(gamma),  0],
                   [         0,           0,  1]])

    return rz

def homo_transxyz(dx, dy, dz):
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans

def homo_rotxyz(alpha, beta, gamma):
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rotx(alpha).dot(roty(beta)).dot(rotz(gamma))
    return rot4x4

def homo_transform(alpha,beta,gamma,dx,dy,dz):
    # Create a homogeneous 4x4 transformation matrix
    return np.dot(homo_transxyz(dx,dy,dz), homo_rotxyz(alpha, beta, gamma))

def homo_inv(matrix):
#    Return the inverse of a homogeneous transformation matrix.
#
#                 -------------------------
#                 |           |           |
#    inverse   =  |    R^T    |  -R^T * d |
#                 |___________|___________|
#                 | 0   0   0 |     1     |
#                 -------------------------

    inverse = matrix
    inverse[:3,:3] = inverse[:3,:3].T # R^T
    inverse[:3,3] = -np.dot(inverse[:3,:3],inverse[:3,3]) # -R^T * d
    return inverse
