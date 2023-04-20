#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *
import numpy as np
from rospy.numpy_msg import numpy_msg

# ----------------- TODO ----------------
# Limit velocity based on threshold (similar to C++)

# ---------------- CLASS ----------------
class ElfinActionNode():
    def __init__(self):
        print('ElfinActionNode: initializing node')

        self.first_read = True
        self.ready = False
        self.joint_states = None

        # Initialize Variables
        self.joint_upper_limit = np.array([ 3.14,  2.35,  2.61,  3.14,  2.56,  3.14])
        self.joint_lower_limit = np.array([-3.14, -2.35, -2.61, -3.14, -2.56, -3.14])
        self.joint_position = np.array([0, 0, 0, 0, 0, 0])
        self.joint_velocity = np.array([0, 0, 0, 0, 0, 0])
        self.joint_velocity_limit = np.array([1.57, 1.57, 1.57, 1.57, 1.57, 1.57])*0.25
        self.joint_names = [
            'elfin_joint1', 
            'elfin_joint2', 
            'elfin_joint3', 
            'elfin_joint4', 
            'elfin_joint5', 
            'elfin_joint6'
        ]
        self.exec_time = 0.1
        self.previous_joint_position = np.array([0, 0, 0, 0, 0, 0])
        self.joint_position_threshold = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])

        # ROS Infrastructure
        topic = 'joint_cmd'
        self.sub_joint_cmd = rospy.Subscriber(topic, numpy_msg(JointState), self.joint_cmd_callback)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        # Action Library for ros_control
        namespace = 'elfin_arm_controller'
        self.robot_client = actionlib.SimpleActionClient(namespace + '/follow_joint_trajectory', FollowJointTrajectoryAction)

        print('Waiting for server...')
        self.robot_client.wait_for_server()
        print('Connected to server')

        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.joint_names

        rospy.sleep(1)


    def joint_cmd_callback(self, msg):
        # Set initial execution time
        exec_time = self.exec_time

        # Extract data from message
        unlimited_joints = msg.position

        # Limit position
        self.joint_position = self.limit_joints(unlimited_joints)

        if (self.first_read):
            self.previous_joint_position = self.joint_states.position
            self.first_read = False

        # SECURITY STUFF TO LIMIT VELOCITY IF GOAL IS TOO FAR AWAY (avoids big movements at great speed)
        if (np.abs(self.joint_position - self.previous_joint_position) > self.joint_position_threshold).any():
            # Calculate time to achieve using max velocity
            times = np.abs(self.joint_position - self.previous_joint_position) / self.joint_velocity_limit
            
            # If greater than max velocities (or soft limits), modify the execution time 
            if (times > exec_time).any():
                pass
                exec_time = times.max()
                rospy.logwarn("EXECUTION TIME ADJUSTED TO %.2f", exec_time)
        # END SECURITY STUFF

        # Update previous joints
        self.previous_joint_position = self.joint_position.copy()

        # Send goal
        self.robot_client.cancel_goal()
        self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(exec_time))]
        self.robot_client.send_goal(self.g)
        self.robot_client.wait_for_result()


    def joint_states_callback(self, msg):
        self.joint_states = msg
        self.ready = True


    def limit_joints(self, q):
        limited = copy.copy(q)
        for i in range(6):
            if q[i] > self.joint_upper_limit[i]:
                limited[i] = self.joint_upper_limit[i]
            elif q[i] < self.joint_lower_limit[i]:
                limited[i] = self.joint_lower_limit[i]
        return limited

if __name__ == '__main__':
    print('starting elfin_action')
    rospy.init_node('elfin_action', disable_signals=True)
    node = ElfinActionNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')