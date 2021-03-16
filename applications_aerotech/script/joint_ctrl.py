#!/usr/bin/env python

import time
import rospy
from numpy import pi

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

name_A = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']

def joint_cb(data):
    print(data)
    print(type(data))
    print("\n\n\n")

def joint_subscriber():
    pose_sub = rospy.Subscriber('/joint_states', JointState) #, joint_cb)
    # rospy.spin()


if __name__ == "__main__":

    rospy.init_node('joint_pos_ctrl', anonymous=True)
    rate = rospy.Rate(10)

    try:
        # joint_subscriber()
        ctrl = rospy.Publisher('/joint_states', JointState, queue_size=10)

        pos_d = JointState()
        pos_d.name = name_A
        pos_d.position = [0, pi/2, 0, pi/2, 0, 0]

        while not rospy.is_shutdown():
            ctrl.publish(pos_d)
            rate.sleep()

    except KeyboardInterrupt:
        print("Stopping controller...")
