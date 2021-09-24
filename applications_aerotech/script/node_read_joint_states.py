#!/usr/bin/env python3

import rospy
import sys
import numpy as np

import sensor_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg

log_joints_desired = []
log_joints_actual = []
log_joints_error = []
log_timestamp = []

def callback(joint_states):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", joint_states.position)
    # log_joints.append([j for j in joint_states.position])
    log_joints_desired.append([j for j in joint_states.feedback.desired.positions])
    log_joints_actual.append([j for j in joint_states.feedback.actual.positions])
    log_joints_error.append([j for j in joint_states.feedback.error.positions])
    log_timestamp.append([joint_states.status.goal_id.stamp])

def listener():

    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, callback)
    rospy.Subscriber("/position_trajectory_controller/follow_joint_trajectory/feedback", control_msgs.msg.FollowJointTrajectoryActionFeedback, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

    print("SALVANDO MINHA LISTA")

    with open('joints_statedesired.txt', 'w') as fd:
        fd.write('a1, a2, a3, a4, a5, a6\n')
        for item in log_joints_desired:
            string_write = str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + ',' + str(item[3]) + ',' + str(item[4]) + ',' + str(item[5]) + '\n'
            fd.write(str(string_write))
    with open('joints_stateactual.txt', 'w') as fa:
        fa.write('a1, a2, a3, a4, a5, a6\n')
        for item in log_joints_actual:
            string_write = str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + ',' + str(item[3]) + ',' + str(item[4]) + ',' + str(item[5]) + '\n'
            fa.write(str(string_write))
    with open('joints_stateerror.txt', 'w') as fe:
        fe.write('a1, a2, a3, a4, a5, a6\n')
        for item in log_joints_error:
            string_write = str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + ',' + str(item[3]) + ',' + str(item[4]) + ',' + str(item[5]) + '\n'
            fe.write(str(string_write))
    with open('joints_timestamp.txt', 'w') as ft:
        ft.write('time\n')
        for item in log_timestamp:
            string_write = str(item) + '\n'
            ft.write(str(string_write))

    fd.close()
    fe.close()
    fa.close()
    ft.close()
    print("SALVO")
