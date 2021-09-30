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

    def local_foot_pos(self, x,y,z,ht):
        global_pos = np.array([x,y,z,1])
        ht_inv = tf.homo_inv(ht) # hip coord
        local_pos = ht_inv.dot(global_pos)
        return local_pos[0:3]

#    def getLegPoint(self, leg,l,w,roll,pitch,yaw,dx,dy,dz):
#        (q1,q2,q3,ht) = self.calculateAngles(self,l,w,roll,pitch,yaw,dx,dy,dz,leg)
#
#        p1 = ht[0:3,3] # world_to_hip
#
#        ht = np.matmul(np.matmul(ht, geo.t0_to_t1(q1,l1)), geo.t1_to_t2())
#        p2 = ht[0:3,3]
#
#        ht = np.matmul(ht, geo.t2_to_t3(q2,l2))
#        p3 = ht[0:3,3]
#
#        ht = np.matmul(ht, geo.t3_to_t4(q3,l3))
#        p4 = ht[0:3,3]
#
#        return (p1,p2,p3,p4)

    def calculateAngles(self,roll,pitch,yaw,dx,dy,dz,leg,l=None,w=None):

        endPoint = np.array([ [ self.bodyLength/2,  self.bodyWidth/2 + self.l1, 0 ],  # FL
                              [-self.bodyLength/2,  self.bodyWidth/2 + self.l1, 0 ],  # RL
                              [-self.bodyLength/2, -self.bodyWidth/2 - self.l1, 0 ],  # RR
                              [ self.bodyLength/2, -self.bodyWidth/2 - self.l1, 0 ]]) # FR
        if l == None:
            l = self.bodyLength
        if w == None:
            w = self.bodyWidth
        t_m = geo.world_to_body(roll,pitch,yaw,dx,dy,dz)
        if leg == 'FL':
            ht = geo.world_to_FL1(t_m,l,w)
            local_pos = self.local_foot_pos(endPoint[0,0], endPoint[0,1], endPoint[0,2],ht)
            (q1,q2,q3) = self.invk(leg,local_pos[0], local_pos[1], local_pos[2])

        elif leg == 'RL':
            ht = geo.world_to_RL1(t_m,l,w)
            local_pos = self.local_foot_pos(endPoint[1,0], endPoint[1,1], endPoint[1,2],ht)
            (q1,q2,q3) = self.invk(leg,local_pos[0], local_pos[1], local_pos[2])

        elif leg == 'RR':
            ht = geo.world_to_RR1(t_m,l,w)
            local_pos = self.local_foot_pos(endPoint[2,0], endPoint[2,1], endPoint[2,2],ht)
            (q1,q2,q3) = self.invk(leg,local_pos[0], local_pos[1], local_pos[2])

        elif leg == 'FR':
            ht = geo.world_to_FR1(t_m,l,w)
            local_pos = self.local_foot_pos(endPoint[3,0], endPoint[3,1], endPoint[3,2],ht)
            (q1,q2,q3) = self.invk(leg,local_pos[0], local_pos[1], local_pos[2])

        return (q1,q2,q3,ht)

    def invk(self, leg,x,y,z):
        # Computes inverse kinematics
        # Input: End-effector position
        # Output: angles

        F=sqrt(x**2+y**2-self.l1**2)
        H=sqrt(F**2+z**2)

        theta1 = atan2(y,x) + atan2(F,self.l1)

        D=(H**2-self.l2**2-self.l3**2)/(2*self.l2*self.l3)
        if leg[1] == 'L':
            theta3 = acos(D)
        elif leg[1] == 'R':
            theta3 = -acos(D)

        theta2=atan2(z,F)-atan2(self.l3*sin(theta3),self.l2+self.l3*cos(theta3))

        return(theta1,theta2,theta3)

    def stand(self,roll=0,pitch=0,yaw=0,dx=0,dy=0,dz=None):
        if dz == None:
            dz = self.bodyHeight

        order = ['FL','RR','FR','RL']
        raw = {'q1': [], 'q2': [], 'q3': []}
        angles = []

        for leg in order:
            (q1,q2,q3,ht) = self.calculateAngles(roll,pitch,yaw,dx,dy,dz,leg)
            raw['q1'].append(q1)
            raw['q2'].append(q2)
            raw['q3'].append(q3)

        for x in raw['q1']:
            angles.append(x)

        for x in raw['q2']:
            angles.append(x)

        for x in raw['q3']:
            angles.append(x)

        return angles

