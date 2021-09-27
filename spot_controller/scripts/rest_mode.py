#!/usr/bin/env python
#encoding: utf-8

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
from IK_solver import IK

class RestMode:
    def __init__(self, bodyDimensions, legDimensions):
        self.rate = rospy.Rate(10.0) #10Hz
        self.rb = IK(bodyDimensions, legDimensions)

#        angle_cmd = ['/spot_controller/FL1_joint/command',
#                     '/spot_controller/RR1_joint/command',
#                     '/spot_controller/FR1_joint/command',
#                     '/spot_controller/RL1_joint/command',
#                     '/spot_controller/FL2_joint/command',
#                     '/spot_controller/RR2_joint/command',
#                     '/spot_controller/FR2_joint/command',
#                     '/spot_controller/RL2_joint/command',
#                     '/spot_controller/FL3_joint/command',
#                     '/spot_controller/RR3_joint/command',
#                     '/spot_controller/FR3_joint/command',
#                     '/spot_controller/RL3_joint/command' ]
        self.joint = []
#        for i in range(12):
#            self.joint.append(rospy.Publisher(angle_cmd[i], Float64, queue_size=10))
        rospy.Subscriber('spot_keyboard/body_pose', Vector3, self.run)

    def run(self,msg):
        while not rospy.is_shutdown():
            try:
                rospy.loginfo([msg])

#            roll  = msg.x
#            pitch = msg.y
#            yaw   = msg.z

#            angles = self.rb.stand(roll,pitch,yaw)
#            for i in range(12):
#               self.joint[i].publish(angles[i])

            except rospy.ROSInterruptException:
                pass

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('rest_mode', anonymous=True)
    body = [0.1908, 0.080, 0.15]
    legs = [0.04, 0.1, 0.094333]
    rest = RestMode(body, legs)













