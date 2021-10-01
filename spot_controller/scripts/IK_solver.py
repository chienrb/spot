#!/usr/bin/env python
#encoding: utf-8

from math import sin, cos, pi, sqrt, atan2, acos, asin
import numpy as np
import transformation as tf
from math import sin, cos, pi, sqrt
import robot_figure as geo

class IK:
    def __init__(self, bodyDimensions, legDimensions):
        self.bodyLength = bodyDimensions[0]
        self.bodyWidth  = bodyDimensions[1]
        self.bodyHeight = bodyDimensions[2]

        self.l1 = legDimensions[0]
        self.l2 = legDimensions[1]
        self.l3 = legDimensions[2]

    def get_local_pos(self,global_pos,roll,pitch,yaw,dx,dy,dz=None,l=None,w=None):
        global_pos = (np.block([ [global_pos         ],
                                 [np.array([1,1,1,1])] ])).T
#        print(global_pos)
        if l == None:
            l = self.bodyLength
        if w == None:
            w = self.bodyWidth
        if dz == None:
            dz = self.bodyHeight

        t_m = geo.world_to_body(roll,pitch,yaw,dx,dy,dz)
#        blwbl = np.dot(t_m, global_pos.T)
#        print(blwbl)

        wFL1 = geo.world_to_FL1(t_m,l,w)
        wRR1 = geo.world_to_RR1(t_m,l,w)
        wFR1 = geo.world_to_FR1(t_m,l,w)
        wRL1 = geo.world_to_RL1(t_m,l,w)

        # local coords
        pos_FL = tf.homo_inv(wFL1).dot(global_pos[0])
        pos_RR = tf.homo_inv(wRR1).dot(global_pos[1])
        pos_FR = tf.homo_inv(wFR1).dot(global_pos[2])
        pos_RL = tf.homo_inv(wRL1).dot(global_pos[3])
#        print(np.array([pos_FL[:3],pos_RR[:3],pos_FR[:3],pos_RL[:3]]))

        return np.array([pos_FL[:3],pos_RR[:3],pos_FR[:3],pos_RL[:3]])

    def invk(self,x,y,z,i):
        # Computes inverse kinematics
        # Input: End-effector position
        # Output: angles
        F=sqrt(x**2+y**2-self.l1**2)
        H=sqrt(F**2+z**2)

        if i == 1 or i == 2:
            theta1 =  atan2(y,x) + atan2(F,self.l1)
        else:
            theta1 = -atan2(y,x) - atan2(F,self.l1)

        D=(H**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)
        theta3 = -acos(D)

        theta2=atan2(z,F)-atan2(self.l3*sin(theta3),self.l2+self.l3*cos(theta3))

        return [theta1,theta2,theta3]

    def getAngles(self, local_pos):
        angles = []
        for i in range(4):
            [q1,q2,q3] = self.invk(local_pos[i,0], local_pos[i,1], local_pos[i,2],i)
            angles.append([q1,q2,q3]) # FL --> RR --> FR --> RL

        angles = np.asarray(angles).T.flatten()
#        print(angles)
        return angles

