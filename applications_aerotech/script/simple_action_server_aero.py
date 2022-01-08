#!/usr/bin/env python3

import rospy
import sys
# from __future__ import print_function
import numpy as np
from numpy import pi
# import roboticstoolbox as rtb
import rtb_kuka_kr16 as kr16
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the

# goal message and the result message.
import std_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg


def joint_client(pointdesired, dt=3):
    goal = control_msgs.msg.FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = rospy.get_param("/controller_joint_names")
    # print(goal.trajectory.joint_names)
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    # point.positions = [0, 0, 0, 0, 0, 0]
    # point.positions = [0, -3.14159/2, 3.14159/2, 0, 3.14159/2, 0]
    point.positions = pointdesired
    # print(dt)
    point.time_from_start = rospy.Duration(dt)  # 3)

    print("goal published")

    goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

    return 0


if __name__ == '__main__':
    rospy.init_node('joint_client_py')
    client = actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory',
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()
    timestep = 0.012
    time = 1.5
    print("server responded!")
    kuka = kr16.KinematicsKR16([-0.512, 0.036, pi, pi])
    # parameters: rtb.RevoluteDH(d=-0.512, a=0.036, alpha=pi, offset=pi)
    print(kuka)
    # rospy.wait_for_message("/activate", std_msgs.msg.String)

    # initial position:
    qi = np.array([0, - pi / 2, pi / 2, 0, pi * 105 / 180, 0])  # with tool
    # qi = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, 0])  # withouttool
    # qi = np.array([0, - 1.53846805, 1.53846805, 0, np.pi/2, 0])

    # qi = np.array([0, - 1.53846805, 1.884224543, 0, 0.48, 0])
    # qi = np.array([0, - 1.53846805, 1.884224543, 0, 0.48, 1.57079500])

    joint_client(qi)
    # qi = np.array([0, - 1.53846805,  1.53846805, 0, np.pi/2, -np.pi/3])
    qm = qi
    qi = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, -np.pi / 3])
    joint_client(qi)
    [ti, Ri] = kuka.fk_kr16(qi)
    qn = np.zeros(6)
    tn = np.zeros(3)
    # cartesian_moves = np.zeros([3, 3])
    cartesian_moves = np.zeros([7, 3])
    # cartesian_moves[0] = [0, 0, -0.1]
    # cartesian_moves[0] = [0, 0, 0.1]
    # cartesian_moves[0] = [0, 0, 0.1]

    ### Movimentações sendo utilizadas
    cartesian_moves[0] = [0, 0, -0.2]
    cartesian_moves[1] = [0, 0.45, 0]
    cartesian_moves[2] = [0.4, 0, 0]
    cartesian_moves[3] = [0, -0.4, 0]
    cartesian_moves[4] = [-0.4, 0, 0]
    cartesian_moves[5] = [0, -0.05, 0]
    cartesian_moves[6] = [0, 0, 0.2]
    for i in np.arange(cartesian_moves.shape[0]):
        # td = ti + cartesian_moves[i]
        # kuka.traj_cart_generate(td, Ri, ti, Ri, timestep, time)
        # for j in np.arange(time/timestep):
        #     [tm, _, _, quat_d, _, _, Rm] = kuka.traj_cart_get_point()
        #     qm = kuka.ik_kr16(tm, Rm, qi)
        #     joint_client(qm, timestep)
        #     print(qm)
        #     # if (tm == td).all():
        #         # print("j is equal", j)
        #         # break
        # # print(qi)
        # qi = qm
        # ti = td
        ###
        t_delta = cartesian_moves[i] * timestep / time
        for j in np.arange(time / timestep):
            [qn, tn] = kuka.cartesian_change(qi, t_delta)
            joint_client(qn, timestep)
            qi = qn
        T_compare = kuka.fkine(qi)
        if ((T_compare.t == cartesian_moves[i]).all()):
            print("Success")

        # [qn, tn] = kuka.cartesian_change(qi, cartesian_moves[i])
        # # print(qi)
        # result = joint_client(qn)
        # qi = qn

    # qm[5] = 0
    # qm = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, 0])
    joint_client(qm)

    print("Done!")
