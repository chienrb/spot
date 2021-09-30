#!/usr/bin/env python
#encoding: utf-8

import rospy
from std_msgs.msg import Float64
import numpy as np
from IK_solver import IK

class StandMode:
    def __init__(self, bodyDimensions, legDimensions):

        self.rate = rospy.Rate(10.0) #10Hz
        self.rb = IK(bodyDimensions, legDimensions)

        angle_cmd = ['spot_controller/FL1_joint/command',
                     'spot_controller/RR1_joint/command',
                     'spot_controller/FR1_joint/command',
                     'spot_controller/RL1_joint/command',
                     'spot_controller/FL2_joint/command',
                     'spot_controller/RR2_joint/command',
                     'spot_controller/FR2_joint/command',
                     'spot_controller/RL2_joint/command',
                     'spot_controller/FL3_joint/command',
                     'spot_controller/RR3_joint/command',
                     'spot_controller/FR3_joint/command',
                     'spot_controller/RL3_joint/command' ]
        self.joint = []
        for i in range(12):
            self.joint.append(rospy.Publisher(angle_cmd[i], Float64, queue_size=10))

        self.run()

    def run(self):
        angles = self.rb.stand()
        for i in range(12):
            self.joint[i].publish(angles[i])

        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('stand_mode', anonymous=True)
    body = [0.1908, 0.080, 0.15]
    legs = [0.04, 0.1, 0.094333]
    try:
        while not rospy.is_shutdown():
            stand = StandMode(body, legs)

    except rospy.ROSInterruptException:
        pass


