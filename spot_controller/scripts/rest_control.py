#!/usr/bin/env python
#encoding: utf-8

import rospy
import numpy as np
from IK_solver import IK

class RestController:
    def __init__(self, body, legs, default_stance):
        self.def_stance = default_stance
        self.body_ori   = [ 0, 0,  0        ]
#        self.body_trans = [ 0, 0, -body[2] ]
        self.bodyLength = body[0]
        self.bodyWidth  = body[1]
        self.bodyHeight = body[2]
        self.rb = IK(body, legs)

    def updateOri(self, msg):
        self.body_ori[0] = msg.x
        self.body_ori[1] = msg.y
        self.body_ori[2] = msg.z

    def run(self):
        local_pos = self.rb.get_local_pos(self.def_stance,self.body_ori[0],self.body_ori[1],self.body_ori[2],0,0)
        angles = self.rb.getAngles(local_pos)

        return angles
