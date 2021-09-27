#!/usr/bin/env python

from math import sin, cos, pi, sqrt, atan2, acos, asin
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import robot_figure as geo
import transformation as tf

bodyLength = 0.1908
bodyWidth  = 0.08
bodyHeight = 0.15

d2r = pi/180
r2d = 180/pi

l1 = 0.04
l2 = 0.1
l3 = 0.094333

# Gazebo coord -> Hip corord -> Angles
# Gazebo coord
global_endPoint = None

ax = None

# positions of each corner
body_X = []; body_Y = []; body_Z = []

# positions of each joint in each leg
joint_X = []; joint_Y = []; joint_Z = []

lines = []
plt_colors = ['r','c','b']

def setupView():
    global ax
    # Attaching 3D axis to the figure
    fig = plt.figure()
    ax = p3.Axes3D(fig, auto_add_to_figure=False)
    fig.add_axes(ax)

    ax.set_xlabel('body_X')
    ax.set_ylabel('body_Z')
    ax.set_zlabel('body_Y')

    ax.set_xlim3d([-0.2, 0.2])
    ax.set_zlim3d([   0, 0.2])
    ax.set_ylim3d([-0.2, 0.2])

def IK(x,y,z,leg):
    # Input: End-effector position
    # Output: angles

    F=sqrt(x**2+y**2-l1**2)
    H=sqrt(F**2+z**2)

    theta1 = atan2(y,x) + atan2(F,l1)
    
    D=(H**2-l2**2-l3**2)/(2*l2*l3)
    if leg[1] == 'L':
        theta3 = acos(D)
    elif leg[1] == 'R':
        theta3 = -acos(D)

    theta2=atan2(z,F)-atan2(l3*sin(theta3),l2+l3*cos(theta3))

    return(theta1,theta2,theta3)

def global_to_local(x,y,z,ht):
    global_pos = np.array([x,y,z,1])
    ht_inv = tf.homo_inv(ht) # hip coord
    local_pos = ht_inv.dot(global_pos)
    return local_pos[0:3]

def bezier_curve(P0,P1,P2,P3):
    # B(t) = P0(1-t)^3 + 3*P1*t*(1-t)^2 + 3*P2*t^2*(1-t) + P3*t^3; t thuoc [0,1]
    
    t = np.linspace(0,1,100)
    xE = np.zeros(100)
    zE = np.zeros(100)
    yE = np.zeros(100)    

    for i in range(100):
        raw = P0*(1-t[i])**3 + 3*P1*t[i]*(1-t[i])**2 + 3*P2*(1-t[i])*t[i]**2 + P3*t[i]**3
        xE[i] = raw[0,0]
        yE[i] = raw[1,0]
        zE[i] = raw[2,0]

    return (xE,yE,zE)

def get_global_endPoint(x,y,z):
    global global_endPoint
    global_endPoint = np.array([[x + bodyLength/2, y + bodyWidth/2 + l1, z ],  # FL
                                [x - bodyLength/2, y + bodyWidth/2 + l1, z ],  # RL
                                [x - bodyLength/2, y - bodyWidth/2 - l1, z ],  # RR
                                [x + bodyLength/2, y - bodyWidth/2 - l1, z ]]) # FR

def getLegPoint(leg,l,w,roll,pitch,yaw,dx,dy,dz):
    # P0=[-0.02;-0.18]; P1=[-0.035;-0.15]; P2=[0.01;-0.165]; P3=[0.02;-0.18];
    P0 = np.array([[ -0.04], [0], [    0]]) 
    P1 = np.array([[-0.055], [0], [0.035]])
    P2 = np.array([[  0.03], [0], [ 0.02]])
    P3 = np.array([[  0.04], [0], [    0]])

    (xE,yE,zE) = bezier_curve(P0,P1,P2,P3)
    p1 = []; p2 = []; p3 = []; p4 = []

    for i in range(100):
        get_global_endPoint(xE[i],yE[i],zE[i])
        t_m = geo.world_to_body(roll,pitch,yaw,dx,dy,dz)
        if leg == 'FL':
            ht = geo.world_to_FL1(t_m,l,w)
            local_pos = global_to_local(global_endPoint[0,0], global_endPoint[0,1], global_endPoint[0,2],ht)
            (q1,q2,q3) = IK(local_pos[0], local_pos[1], local_pos[2], leg)

        elif leg == 'RL':
            ht = geo.world_to_RL1(t_m,l,w)
            local_pos = global_to_local(global_endPoint[1,0], global_endPoint[1,1], global_endPoint[1,2],ht)
            (q1,q2,q3) = IK(local_pos[0], local_pos[1], local_pos[2], leg)

        elif leg == 'RR':
            ht = geo.world_to_RR1(t_m,l,w)
            local_pos = global_to_local(global_endPoint[2,0], global_endPoint[2,1], global_endPoint[2,2],ht)
            (q1,q2,q3) = IK(local_pos[0], local_pos[1], local_pos[2], leg)

        elif leg == 'FR':
            ht = geo.world_to_FR1(t_m,l,w)
            local_pos = global_to_local(global_endPoint[3,0], global_endPoint[3,1], global_endPoint[3,2],ht)
            (q1,q2,q3) = IK(local_pos[0], local_pos[1], local_pos[2], leg)

        p1.append(list(ht[0:3,3]))

        #    ht = ht @ geo.t0_to_t1(q1,l1) @ geo.t1_to_t2() # world_to_femur
        ht = np.matmul(np.matmul(ht, geo.t0_to_t1(q1,l1)), geo.t1_to_t2())
        p2.append(list(ht[0:3,3]))

        #    ht = ht @ geo.t2_to_t3(q2,l2) # world_to_coxa
        ht = np.matmul(ht, geo.t2_to_t3(q2,l2))
        p3.append(list(ht[0:3,3]))

        #    ht = ht @ geo.t3_to_t4(q3,l3) # world_to_foot
        ht = np.matmul(ht, geo.t3_to_t4(q3,l3))
        p4.append(list(ht[0:3,3]))

    p1 = np.array(p1); p2 = np.array(p2); p3 = np.array(p3); p4 = np.array(p4)

    return (p1,p2,p3,p4)

def posJoint(l,w,roll=0,pitch=0,yaw=0,dx=0,dy=0,dz=bodyHeight):
    global joint_X, joint_Y, joint_Z, body_X, body_Y, body_Z
    order = ['FL','RL','RR','FR']
    for leg in order:
        (p1,p2,p3,p4) = getLegPoint(leg,bodyLength,bodyWidth,roll,pitch,yaw,dx,dy,dz)

        body_X.append(p1[0,0])
        body_Y.append(p1[0,1])
        body_Z.append(p1[0,2])
        for i in range(100):
            joint_X.append([ p1[i,0], p2[i,0], p3[i,0], p4[i,0] ])
            
            joint_Y.append([ p1[i,1], p2[i,1], p3[i,1], p4[i,1] ])
            
            joint_Z.append([ p1[i,2], p2[i,2], p3[i,2], p4[i,2] ])

            draw_leg()
            joint_X = []; joint_Y = []; joint_Z = []

    joint_X = None; joint_Y = None; joint_Z = None

    body_X.append(body_X[0])
    body_Y.append(body_Y[0])
    body_Z.append(body_Z[0])
    draw_body()

    body_X = None; body_Y = None; body_Z = None

def draw_leg():
    for i in range(3):
        x_vals = [ joint_X[0][i], joint_X[0][i+1] ]
        y_vals = [ joint_Y[0][i], joint_Y[0][i+1] ]
        z_vals = [ joint_Z[0][i], joint_Z[0][i+1] ]
        lines.append(ax.plot(x_vals,y_vals,z_vals,color=plt_colors[i]))

def draw_body():
    for i in range(4):
        x_vals = [ body_X[i], body_X[i+1] ]
        y_vals = [ body_Y[i], body_Y[i+1] ]
        z_vals = [ body_Z[i], body_Z[i+1] ]
        lines.append(ax.plot(x_vals,y_vals,z_vals,color='r'))

if __name__ == '__main__':
    setupView()

    posJoint(bodyLength,bodyWidth)

    plt.show()
