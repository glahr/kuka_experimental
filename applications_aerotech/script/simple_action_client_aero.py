#!/usr/bin/env python2.7

import rospy
import sys
# from __future__ import print_function
import numpy as np
from numpy import pi

# Brings in the SimpleActionClient
import actionlib

import std_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg


def joint_client(pointdesired, dt=3):
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = rospy.get_param("/controller_joint_names")
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = pointdesired
    point.time_from_start = rospy.Duration(dt)

    print("goal published")

    goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

    return 0

if __name__ == '__main__':
    rospy.init_node('joint_client_py')
    client = actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()
    timestep = 0.12
    time = 3
    print("server responded!")

    qi = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, 0])

    joint_client(qi)
    qi = np.array([0, - 1.53846805,  1.884224543, 0, 1.26701762, - 1.57079500])
    joint_client(qi)

    qm = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, 0])
    joint_client(qm)

    qm = np.array([0, -pi / 2, pi / 2, 0, pi / 2, 0])
    joint_client(qm)

    print("Done!")
