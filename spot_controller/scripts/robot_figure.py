#!/usr/bin/env python
# encoding: utf-8

import transformation as tf
from math import sin, cos, pi
import numpy as np

#    Args:
#        t_m: 4x4 numpy matrix. Homogeneous transform representing the coordinate system of the center
#        of the robot body: gazebo frame
#        l: length of the robot body
#        w: width of the robot body

def world_to_body(roll,pitch,yaw,dx,dy,dz): #t_m
    return tf.homo_transform(roll,pitch,yaw,dx,dy,dz)

def world_to_FL1(t_m,l,w):
    matrix = tf.homo_transform(pi/2, pi/2, 0, l/2,  w/2, 0)
    return np.matmul(t_m,matrix)

def world_to_RL1(t_m,l,w):
    matrix = tf.homo_transform(pi/2, pi/2, 0,-l/2,  w/2, 0)
    return np.matmul(t_m,matrix)

def world_to_RR1(t_m,l,w):
    matrix = tf.homo_transform(pi/2, -pi/2, 0,-l/2, -w/2, 0)
    return np.matmul(t_m,matrix)

def world_to_FR1(t_m,l,w):
    matrix = tf.homo_transform(pi/2, -pi/2, 0, l/2, -w/2, 0)
    return np.matmul(t_m,matrix)

def t0_to_t1(theta1, l1):
    t_01 = np.array([[ cos(theta1), -sin(theta1), 0, l1*cos(theta1) ],
                     [ sin(theta1),  cos(theta1), 0, l1*sin(theta1) ],
                     [           0,            0, 1,              0 ],
                     [           0,            0, 0,              1 ]])
    return t_01

def t1_to_t2():
    t_12 = np.array([[ 0,  0, -1,  0],
                     [-1,  0,  0,  0],
                     [ 0,  1,  0,  0],
                     [ 0,  0,  0,  1]])
    return t_12

def t2_to_t3(theta2, l2):
    t_23 = np.array([[ cos(theta2), -sin(theta2), 0, l2*cos(theta2) ],
                     [ sin(theta2),  cos(theta2), 0, l2*sin(theta2) ],
                     [           0,            0, 1,              0 ],
                     [           0,            0, 0,              1 ]])
    return t_23

def t3_to_t4(theta3, l3):
    t_34 = np.array([[ cos(theta3), -sin(theta3), 0, l3*cos(theta3) ],
                     [ sin(theta3),  cos(theta3), 0, l3*sin(theta3) ],
                     [           0,            0, 1,              0 ],
                     [           0,            0, 0,              1 ]])
    return t_34

def t0_to_t4(theta1, theta2, theta3, l1, l2, l3):
    return t0_to_t1(theta1,l1).dot(t1_to_t2()).dot(t2_to_t3(theta2,l2)).dot(t3_to_t4(theta3,l3))


