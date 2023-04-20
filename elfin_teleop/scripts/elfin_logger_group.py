#!/usr/bin/env python
import os
import time
import roslib#; roslib.load_manifest('ur_driver')
import rospy
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------- CLASS ----------------
class ElfinLoggerGroupNode():
    def __init__(self):
        print("ElfinLoggerGroupNode: initializing node")

        # Initialize folders
        logs_folder = os.path.join("logs", "group")
        directory = os.path.join(os.path.abspath(os.path.dirname(__file__)), logs_folder)

        if not os.path.exists(directory):
            os.makedirs(directory)

        # Initialize variables
        self.goal_position_filename = os.path.join(directory, "goal_position.txt")
        self.joint_states_position_filename = os.path.join(directory, "joint_states_position.txt")
        self.joint_states_velocity_filename = os.path.join(directory, "joint_states_velocity.txt")
        self.joint_states_effort_filename = os.path.join(directory, "joint_states_effort.txt")
        self.goal_position = np.zeros(6)

        # Initialize files
        self.command_position_file = open(self.goal_position_filename, 'w')
        self.joint_states_position_file = open(self.joint_states_position_filename, 'w')
        self.joint_states_velocity_file = open(self.joint_states_velocity_filename, 'w')
        self.joint_states_effort_file = open(self.joint_states_effort_filename, 'w')

        # ROS Infrastructure
        topic = 'elfin_arm_controller/command'
        self.sub_command = rospy.Subscriber(topic, Float64MultiArray, self.command_callback)

        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)


    def command_callback(self, msg):
        # Extract Information
        self.goal_position = np.array(msg.data)

        # Write into respective file
        # in joint state callback


    def joint_states_callback(self, msg):
        # Extract information
        seconds = msg.header.stamp.to_sec()
        joint_state_position = np.array(msg.position)
        joint_state_velocity = np.array(msg.velocity)
        joint_state_effort = np.array(msg.effort)

        # Write into respective file
        self.joint_states_position_file.write(str(seconds) + '\t' + '\t'.join(joint_state_position.astype(str)) + '\n')
        self.joint_states_velocity_file.write(str(seconds) + '\t' + '\t'.join(joint_state_velocity.astype(str)) + '\n')
        self.joint_states_effort_file.write(str(seconds) + '\t' + '\t'.join(joint_state_effort.astype(str)) + '\n')
        self.command_position_file.write(str(seconds) + '\t' + '\t'.join(self.goal_position.astype(str)) + '\n')


if __name__ == '__main__':
    print('starting elfin_logger_group')
    rospy.init_node('elfin_logger_group', disable_signals=True)
    node = ElfinLoggerGroupNode()
    
    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')