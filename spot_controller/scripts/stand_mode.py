#!/usr/bin/env python
#encoding: utf-8

import rospy
from std_msgs.msg import Float64
import numpy as np
from IK_solver import IK

class StandMode:
    def __init__(self, body, legs):
        self.def_stance = np.array([ [body[0]/2          , -body[0]/2          ,  body[0]/2          , -body[0]/2]          ,
                                     [body[1]/2 + legs[0], -body[1]/2 - legs[0], -body[1]/2 - legs[0],  body[1]/2 + legs[0]],
                                     [0                  ,  0                  ,  0                  ,  0                  ] ])
        self.rate = rospy.Rate(10.0) #10Hz
        self.rb = IK(body, legs)

        angle_cmd = ['/spot_controller/FL1_joint/command',
                     '/spot_controller/RR1_joint/command',
                     '/spot_controller/FR1_joint/command',
                     '/spot_controller/RL1_joint/command',
                     '/spot_controller/FL2_joint/command',
                     '/spot_controller/RR2_joint/command',
                     '/spot_controller/FR2_joint/command',
                     '/spot_controller/RL2_joint/command',
                     '/spot_controller/FL3_joint/command',
                     '/spot_controller/RR3_joint/command',
                     '/spot_controller/FR3_joint/command',
                     '/spot_controller/RL3_joint/command']
        self.joint = []
        for i in range(12):
            self.joint.append(rospy.Publisher(angle_cmd[i], Float64, queue_size=10))

        self.run()

    def run(self):
        local_pos = self.rb.get_local_pos(self.def_stance,0,0,0,0,0)
        angles = self.rb.getAngles(local_pos)
        for i in range(12):
            self.joint[i].publish(angles[i])

        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('stand_mode', anonymous=True)
    body = [0.2028, 0.080, 0.15]
    legs = [0.04, 0.1, 0.094333]
    try:
        stand = StandMode(body, legs)

    except ROSTimeMovedBackwardsException:
        pass


