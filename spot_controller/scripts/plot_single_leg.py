#!/usr/bin/env python

from math import sin, cos, pi, sqrt, atan2, acos, asin
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

l1 = 0.055
l2 = 0.1075
l3 = 0.13

x4 = 0
y4 = -0.18
z4 = -0.055

ax = None
joint_X = []
joint_Y = []
joint_Z = []

def setupView():
    global ax
    # Attaching 3D axis to the figure
    fig = plt.figure()
#    ax = p3.Axes3D(fig, auto_add_to_figure=False)
    ax = p3.Axes3D(fig)
#    fig.add_axes(ax)

    ax.set_xlabel('joint_X')
    ax.set_zlabel('joint_Y')
    ax.set_ylabel('joint_Z')

    ax.set_xlim3d([-0.15, 0.15])
    ax.set_zlim3d([-0.25, 0.05])
    ax.set_ylim3d([-0.15,0.15])

def IK(x,y,z):
    # Input: End-effector position
    # Output: angles

    F=sqrt(x**2+y**2-l1**2)
    H=sqrt(F**2+z**2)

    theta1=atan2(y,x)+atan2(F,l1)

    D=(H**2-l2**2-l3**2)/(2*l2*l3)
    theta3=acos(D)

    theta2=atan2(z,F)-atan2(l3*sin(theta3),l2+l3*cos(theta3))

    return(theta1,theta2,theta3)

def posJoint(x,y,z):
    global joint_X,joint_Y,joint_Z
    (q1,q2,q3) = IK(x,y,z)

    joint_X = [ 0, l1*cos(q1), l1*cos(q1) + l2*sin(q1)*cos(q2), l1*cos(q1) + l2*sin(q1)*cos(q2) + sin(q1)*l3*cos(q2+q3) ]
    joint_Y = [ 0, l1*sin(q1), l1*sin(q1) - l2*cos(q1)*cos(q2), l1*sin(q1) - l2*cos(q1)*cos(q2) - cos(q1)*l3*cos(q2+q3) ]
    joint_Z = [ 0,          0,                      l2*sin(q2),                              l2*sin(q2) + l3*sin(q2+q3) ]

def draw_leg():
    lines = []
    plt_colors = ['r','c','b']

    for i in range(3):
        x_vals = [ joint_X[i], joint_X[i+1] ]
        y_vals = [ joint_Y[i], joint_Y[i+1] ]
        z_vals = [ joint_Z[i], joint_Z[i+1] ]
        lines.append(ax.plot(x_vals,y_vals,z_vals,color=plt_colors[i]))

    plt.show()

if __name__ == '__main__':
    setupView()
    posJoint(x4,y4,z4)
    draw_leg()

