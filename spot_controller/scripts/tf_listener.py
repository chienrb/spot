#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage

class LatestTF:
    def __init__(self):
        rospy.init_node('tf_listener', anonymous=True)
        self.frame_id = None
        self.child_frame_id = None
        self.translation = None
        self.rotation = None

    def tf_listener(self):
        rospy.Subscriber('tf', TFMessage, self.tf_callback)
        rospy.spin()

    def tf_callback(self, sub):
        print("tf_callback")
        msg = sub.transforms
        self.frame_id = msg[0].header.frame_id
        self.child_frame_id = msg[0].child_frame_id
        self.translation = msg[0].transform.translation
        self.rotation = msg[0].transform.rotation

        print(self.translation.x)

if __name__ == "__main__":
    latesttf = LatestTF()
    latesttf.tf_listener()
