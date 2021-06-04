#!/usr/bin/env python

import rospy
import sys
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import std_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg


def joint_client():
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = rospy.get_param("/controller_joint_names")

    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = [0, 0, 0, 0, 0, 0]
    # point.positions = [0, -3.14159/6, 3.14159/2, 0, 3.14159/2, 0]
    point.time_from_start = rospy.Duration(3)

    print("goal published")

    goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

    return

if __name__ == '__main__':
    rospy.init_node('joint_client_py')
    client = actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()
    print("server responded!")
    result = joint_client()
    print("Done!")
