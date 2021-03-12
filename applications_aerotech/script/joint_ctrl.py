#!/usr/bin/env python

import time
import rospy

from sensor_msgs.msg import JointState

def joint_cb(data):
    print(data)
    print(type(data))
    print("\n\n\n")

def joint_subscriber():
    pose_sub = rospy.Subscriber('/joint_states', JointState) #, joint_cb)
    rospy.spin()

if __name__ == "__main__":

    rospy.init_node('joint_pos_ctrl', anonymous=True)

    try:
        joint_subscriber()
    except KeyboardInterrupt:
        print("Stopping controller...")
