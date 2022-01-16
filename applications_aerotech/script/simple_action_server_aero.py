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


class Tool:
    """Defines tool properties"""
    def __init__(self, name, DH_parameters, work_height, A6_value):
        self.name = name
        self.work_height = work_height
        self.A6_value = A6_value
        self.DHRobot = kr16.KinematicsKR16(DH_parameters)


class Robot:
    """Informs current robot tool"""
    def __init__(self):
        self.tool = Tool('Empty', [-0.512, 0.036, pi, pi], 0, 0)
    def SetTool(self, tool, q_safe):
        self.tool = tool
        q_current = go_to_safe_height(q_safe, self)
        return q_current


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


def cartesian_move(movement_list, q0, time=3):
    movement = np.array(movement_list)
    timestep = 0.012
    t_delta = movement * timestep / time
    for j in np.arange(time / timestep):
        # [qn, tn] = kuka.cartesian_change(q0, t_delta)
        [qn, tn] = machine.tool.DHRobot.cartesian_change(q0, t_delta)
        joint_client(qn, timestep)
        q0 = qn
    T_compare = machine.tool.DHRobot.fkine(q0)
    if ((T_compare.t == movement).all()):
        print("Success")
    print(f"{q0}")

    return q0


def  go_to_work_height(q_safe, robot):
    q_current = cartesian_move([0, 0, robot.tool.work_height], q_safe, 1.5)
    return q_current


def go_to_safe_height(q_safe, robot, time=3):
    q_safe[5] = robot.tool.A6_value
    joint_client(q_safe, time)
    return q_safe



if __name__ == '__main__':
    rospy.init_node('joint_client_py')
    client = actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory',
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()
    timestep = 0.012
    time = 1.5
    print("server responded!")

    # kuka = kr16.KinematicsKR16([-0.512, 0.036, pi, pi])
    # parameters: rtb.RevoluteDH(d=-0.512, a=0.036, alpha=pi, offset=pi)

    ballpoint = Tool('ballpoint', [-0.512, 0.036, pi, pi], -0.033, pi/2)
    marker = Tool('marker', [-0.512, 0.036, pi, pi], -0.038, -pi/2)
    machine = Robot()

    print(machine.tool.DHRobot)

    # rospy.wait_for_message("/activate", std_msgs.msg.String)

    '''initial position in radians for alll 6 joints:'''
    q_initial = np.array([0, - pi / 2, pi / 2, 0, pi * 105 / 180, 0])  # with tool
    # q_initial = np.array([0, - 1.53846805, 1.884224543, 0, 1.26701762, 0])  # withouttool
    joint_client(q_initial)

    '''current joint positions:'''
    q_current = q_initial

    # position at a safe height from experiment:
    q_safe =  np.array([0, -1.56562183,  1.62843914, 0, 1.76977840, 0])

    q_current = go_to_safe_height(q_safe, machine)
    q_current = machine.SetTool(ballpoint, q_safe)

    q_current = go_to_work_height(q_current, machine) # ballpoint = -0.033

    q_current = cartesian_move([0, -0.18, 0], q_current, 1.5)
    q_current = cartesian_move([0.17, 0, 0], q_current, 1.5)
    q_current = cartesian_move([0, 0.17, 0], q_current, 1.5)
    q_current = cartesian_move([-0.17, 0, 0], q_current, 1.5)
    q_current = cartesian_move([0, 0.01, 0], q_current, 1)

    q_current = go_to_safe_height(q_safe, machine, 2)

    q_current = machine.SetTool(marker, q_safe)

    q_current = go_to_work_height(q_current, machine)
    # q_current = cartesian_move([0, 0, -0.038], q_safe, 1.5)
    q_current = cartesian_move([0, -0.18, 0], q_current, 1.5)
    q_current = cartesian_move([0.17, 0, 0], q_current, 1.5)
    q_current = cartesian_move([0, 0.17, 0], q_current, 1.5)
    q_current = cartesian_move([-0.17, 0, 0], q_current, 1.5)
    q_current = cartesian_move([0, 0.01, 0], q_current, 1)
    # #
    # joint_client(q_safe, 2)
    q_current = go_to_safe_height(q_safe, machine, 2)

    print("Done!")

    #this sequence was working
    # [ti, Ri] = kuka.fk_kr16(qi)
    # qn = np.zeros(6)
    # tn = np.zeros(3)
    # # cartesian_moves = np.zeros([3, 3])
    # cartesian_moves = np.zeros([7, 3])
    # cartesian_moves[0] = [0, 0, -0.072]
    # cartesian_moves[1] = [0, 0.19, 0]
    # cartesian_moves[2] = [0.18, 0, 0]
    # cartesian_moves[3] = [0, -0.18, 0]
    # cartesian_moves[4] = [-0.18, 0, 0]
    # cartesian_moves[5] = [0, -0.01, 0]
    # cartesian_moves[6] = [0, 0, 0.03]
    # #
    # for i in np.arange(cartesian_moves.shape[0]):
    #     # td = ti + cartesian_moves[i]
    #     # kuka.traj_cart_generate(td, Ri, ti, Ri, timestep, time)
    #     # for j in np.arange(time/timestep):
    #     #     [tm, _, _, quat_d, _, _, Rm] = kuka.traj_cart_get_point()
    #     #     qm = kuka.ik_kr16(tm, Rm, qi)
    #     #     joint_client(qm, timestep)
    #     #     print(qm)
    #     #     # if (tm == td).all():
    #     #         # print("j is equal", j)
    #     #         # break
    #     # # print(qi)
    #     # qi = qm
    #     # ti = td
    #     ###
    #
    #     # [qn, tn] = kuka.cartesian_change(qi, cartesian_moves[i])
    #     # # print(qi)
    #     # result = joint_client(qn)
    #     # qi = qn
    #
