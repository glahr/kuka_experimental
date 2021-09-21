#!/usr/bin/env python3

import rospy
import sys
import numpy as np

import sensor_msgs.msg

log_joints = []

def callback(joint_states):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", joint_states.position)
    log_joints.append([j for j in joint_states.position])

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

    print("SALVANDO MINHA LISTA")

    with open('joints_statescsv.csv', 'w') as f:
        f.write('a1, a2, a3, a4, a5, a6\n')
        for item in log_joints:
            # f.write(str(item[0]),',',str(item[1]),',',str(item[2]),',',str(item[3]),',',str(item[4]),',',str(item[5]),'\n')
            string_write = str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + ',' + str(item[3]) + ',' + str(item[4]) + ',' + str(item[5]) + '\n'
            f.write(str(string_write))

    print("SALVO")
