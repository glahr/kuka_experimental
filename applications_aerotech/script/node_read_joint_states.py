#!/usr/bin/env python3

import rospy
import sys
import numpy as np

import sensor_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float64, Float64MultiArray

log_joints_desired = []
log_joints_actual = []
log_joints_error = []
log_timestamp = []
log_ft = []


def callback(joint_states):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", joint_states.status.goal_id.stamp)
    # log_joints.append([j for j in joint_states.position])
    log_joints_desired.append([j for j in joint_states.feedback.desired.positions])
    log_joints_actual.append([j for j in joint_states.feedback.actual.positions])
    log_joints_error.append([j for j in joint_states.feedback.error.positions])
    log_timestamp.append([joint_states.status.goal_id.stamp])

    # mqtt_publisher(log_joints_desired[-1], Float64, 'joint_state_desired')
    # mqtt_publisher(log_joints_actual[-1], Float64, 'joint_state_actual')
    # mqtt_publisher(log_joints_actual[-1], Float64, 'joint_state_error')
    mqtt_publisher(joint_states.feedback.desired.positions, Float64MultiArray, 'joint_state_desired')
    mqtt_publisher(joint_states.feedback.actual.positions, Float64MultiArray, 'joint_state_actual')
    mqtt_publisher(joint_states.feedback.error.positions, Float64MultiArray, 'joint_state_error')
    mqtt_publisher(joint_states.feedback, control_msgs.msg.FollowJointTrajectoryFeedback, 'joint_state_feedback')

    # publish timestamp to mqtt -- working!
    mqtt_publisher(str(joint_states.status.goal_id.stamp), String, 'timestamp')


def ft_callback(ft_sensor_data):
    log_ft.append([ft_sensor_data.wrench.force.x, ft_sensor_data.wrench.force.y, ft_sensor_data.wrench.force.z,
                      ft_sensor_data.wrench.torque.x, ft_sensor_data.wrench.torque.y, ft_sensor_data.wrench.torque.z])
    mqtt_publisher(ft_sensor_data, geometry_msgs.msg.WrenchStamped, 'ft_sensor')


def xml_writer(data):
    # rospy.loginfo("I heard %s", data)
    log_xml.append([data])


def listener():

    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, callback)
    rospy.Subscriber("/ft_sensor_data", geometry_msgs.msg.WrenchStamped, ft_callback)
    rospy.Subscriber("/position_trajectory_controller/follow_joint_trajectory/feedback", control_msgs.msg.FollowJointTrajectoryActionFeedback, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# function that publishes string to given topic. topic must be available to send to MQTT
def mqtt_publisher(message, message_type, topic):
    if message_type == Float64MultiArray:
        message_content = message
        message = Float64MultiArray()
        message.data = [float(j) for j in message_content]

    pub = rospy.Publisher(topic, message_type, queue_size=10)
    pub.publish(message)


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
    with open('ft_sensor.txt', 'w') as fft:
        fe.write('Fx, Fy, Fz, Mx, My, Mz\n')
        for item in log_ft:
            string_write = str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + ',' + str(item[3]) + ',' + str(item[4]) + ',' + str(item[5]) + '\n'
            fft.write(str(string_write))

    fd.close()
    fe.close()
    fa.close()
    ft.close()
    fft.close()
    print("SALVO")
