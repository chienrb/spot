#!/usr/bin/env python
#encoding: utf-8

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
from rest_control import RestController

body = [0.2028, 0.080, 0.15]
legs = [0.04, 0.1, 0.094333]

default_stance = np.array([ [body[0]/2          , -body[0]/2          ,  body[0]/2          , -body[0]/2          ],
                            [body[1]/2 + legs[0], -body[1]/2 - legs[0], -body[1]/2 - legs[0],  body[1]/2 + legs[0]],
                            [0                  ,  0                  ,  0                  ,  0                  ] ])

rest = RestController(body, legs, default_stance)

rospy.init_node('robot_control', anonymous=True)

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
    joint.append(rospy.Publisher(angle_cmd[i], Float64, queue_size=10))
rospy.Subscriber('spot_keyboard/body_pose', Vector3, rest.updateOri)

rate = rospy.Rate(10.0) #10Hz

del body
del legs
del default_stance

while not rospy.is_shutdown():
    try:
        angles = rest.run()
#        print(angles)
        for i in range(12):
           joint[i].publish(angles[i])
    except rospy.ROSTimeMovedBackwardsException:
        pass

    rate.sleep()
