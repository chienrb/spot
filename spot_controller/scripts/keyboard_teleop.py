#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import sys, select, termios, tty
from math import pi

class Teleop:
    def __init__(self, rate):
        rospy.init_node('keyboard_teleop')

        self.pose_publisher = rospy.Publisher('spot_keyboard/body_pose', Vector3, queue_size = 10)
        self.rate = rospy.Rate(rate)
        self.msg = """
Reading from the keyboard  and Publishing to Pose!
---------------------------
r: roll angle
p: pitch angle
y: yaw angle
i: increase
d: decrease
q: quit
CTRL-C to quit
--------------------------
        """
        self.body_pose = Vector3()
        self.poll_keys()

    def poll_keys(self):
        print(self.msg)
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            while not rospy.is_shutdown():
                key = self.getKey()

                if key == 'i':
                    print('increase angle: r - roll | p - pitch | y - yaw')
                    while (1):
                        key = self.getKey()
                        if key == 'r':
                            print('increase roll')
                            self.body_pose.x += 2.5*pi/180
                        
                        elif key == 'p':
                            print('increase pitch')
                            self.body_pose.y += 2.5*pi/180
                            
                        elif key == 'y':
                            print('increase yaw')
                            self.body_pose.z += 2.5*pi/180

                        elif key == 'q':
                            break

                        self.pose_publisher.publish(self.body_pose)
                        self.rate.sleep()

                elif key == 'd':
                    print('decrease angle: r - roll | p - pitch | y - yaw')
                    while (1):
                        key = self.getKey()
                        if key == 'r':
                            self.body_pose.x -= 2.5*pi/180
                        
                        elif key == 'p':
                            self.body_pose.y -= 2.5*pi/180
                            
                        elif key == 'y':
                            self.body_pose.z -= 2.5*pi/180

                        elif key == 'q':
                            break

                        self.pose_publisher.publish(self.body_pose)
                        self.rate.sleep()

                elif key == '\x03':
                    break

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

if __name__ == "__main__":
    teleop = Teleop(10.0)
