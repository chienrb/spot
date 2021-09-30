#!/usr/bin/env python
#encoding: utf-8

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
from IK_solver import IK

body = [0.1908, 0.080, 0.15]
legs = [0.04, 0.1, 0.094333]

rb = IK(body, legs)

def run(msg):
    try:
        angles = rb.stand(msg.x, msg.y, msg.z)

        for i in range(12):
           joint[i].publish(angles[i])
    except rospy.ROSInterruptException:
        pass

    rate.sleep()

rospy.init_node('rest_mode', anonymous=True)

rate = rospy.Rate(10.0) #10Hz

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
joint = []
for i in range(12):
    joint.append(rospy.Publisher(angle_cmd[i], Float64, queue_size=0))
rospy.Subscriber('spot_keyboard/body_pose', Vector3, run)

del angle_cmd
del body
del legs

rospy.spin()
