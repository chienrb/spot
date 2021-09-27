#!/usr/bin/env python

from math import sin, cos, pi, sqrt, atan2, acos, asin
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import geometry as geo

bodyLength = 0.14
bodyWidth = 0.11
bodyHeight = 0.15

ax = None
body_X = []
body_Y = []
body_Z = []

def setupView():
    global ax
    # Attaching 3D axis to the figure
    fig = plt.figure()
#    ax = p3.Axes3D(fig, auto_add_to_figure=False)
    ax = p3.Axes3D(fig)
#    fig.add_axes(ax)

    ax.set_xlabel('body_X')
    ax.set_ylabel('body_Z')
    ax.set_zlabel('body_Y')

    ax.set_xlim3d([-0.2, 0.2])
    ax.set_zlim3d([0, 0.4])
    ax.set_ylim3d([-0.2,0.2])

def posHip(l,w):
    global body_X,body_Y,body_Z

    t_m = np.asarray(geo.world_to_body(0,0,0,0,0,bodyHeight))

    # Matrix: 4x4
    FL = np.asarray(geo.world_to_FL1(t_m,l,w)).T[3][:3]
    RL = np.asarray(geo.world_to_RL1(t_m,l,w)).T[3][:3]
    RR = np.asarray(geo.world_to_RR1(t_m,l,w)).T[3][:3]
    FR = np.asarray(geo.world_to_FR1(t_m,l,w)).T[3][:3]

    body_X = [ FL[0], RL[0], RR[0], FR[0], FL[0] ]
    body_Y = [ FL[1], RL[1], RR[1], FR[1], FL[1] ]
    body_Z = [ FL[2], RL[2], RR[2], FR[2], FL[2] ]

def draw_body():
    lines = []

    for i in range(4):
        x_vals = [ body_X[i], body_X[i+1] ]
        y_vals = [ body_Y[i], body_Y[i+1] ]
        z_vals = [ body_Z[i], body_Z[i+1] ]
        lines.append(ax.plot(x_vals,y_vals,z_vals,color='r'))
    # ax.plot(body_X,body_Y,body_Z)
    plt.show()

if __name__ == '__main__':
    setupView()
    posHip(bodyLength, bodyWidth)
    draw_body()

