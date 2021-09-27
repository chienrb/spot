#!/usr/bin.env python
#https://github.com/robo4869/Gait_trajectory/blob/c0455119ac52a2f1f0d54c225aa7a33574d19c9b/Walking_gait/walking_gait_bezier_trajectory.m

import numpy as np
import robot_figure as geo
import transformation as tf
from IK_solver import IK

class WalkGait:
    def __init__(self, legDimensions):
        self.l1 = legDimensions[0]
        self.l2 = legDimensions[1]
        self.l3 = legDimensions[2]
        self.rb = IK(bodyDimensions, legDimensions)

    def bezier_curve(self,P0,P1,P2,P3):
        t = np.linspace(0,1,100);
        xE = np.zeros(100)
        yE = np.zeros(100)

        for i in range(100):
            raw = P0*(1-t[i])**3 + 3*P1*t[i]*(1-t[i])**2 + 3*P2*t[i]**2*(1-t[i]) + P3*t[i]**3;
            xE[i]=raw[0,0];
            yE[i]=raw[1,0];
        
        return (xE,yE)
    
    def calculateAngle(self, x, z, y=0):
        (q1,q2,q3) = self.rb.invk(x,y,z)
        
