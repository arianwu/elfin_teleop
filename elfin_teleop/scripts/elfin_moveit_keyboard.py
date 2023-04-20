#!/usr/bin/env python
import sys
import time
import copy
import rospy
import numpy as np
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

# ---------------- CLASS ----------------
class ElfinMoveItKeyboardNode():
    def __init__(self, commander):
        print('ElfinMoveItKeyboardNode: initializing node')

        # Initialize Variables
        self.joint_upper_limit = [ 3.14,  2.35,  2.61,  3.14,  2.56,  3.14]
        self.joint_lower_limit = [-3.14, -2.35, -2.61, -3.14, -2.56, -3.14]
        self.joint_position = [0, 0, 0, 0, 0, 0]
        self.joint_velocity = [0, 0, 0, 0, 0, 0]
        self.joint_names = [
            'elfin_joint1', 
            'elfin_joint2', 
            'elfin_joint3', 
            'elfin_joint4', 
            'elfin_joint5', 
            'elfin_joint6'
        ]

        # ROS Infrastructure
        topic = "keys"
        self.sub_keys = rospy.Subscriber(topic, String, self.keys_callback)

        # Initialize MoveIt Commander
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "elfin_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)

        rospy.sleep(1)


    def keys_callback(self, msg):
        i = 0.1
        key = msg.data
        unlimited_joints = copy.copy(self.joint_position)
        if(key=="q"):
            unlimited_joints[0] += i
        elif(key=="a"):
            unlimited_joints[0] -= i
        
        elif(key=="w"):
            unlimited_joints[1] += i
        elif(key=="s"):
            unlimited_joints[1] -= i
     
        elif(key=="e"):
            unlimited_joints[2] += i
        elif(key=="d"):
            unlimited_joints[2] -= i
     
        elif(key=="r"):
            unlimited_joints[3] += i
        elif(key=="f"):
            unlimited_joints[3] -= i
     
        elif(key=="t"):
            unlimited_joints[4] += i
        elif(key=="g"):
            unlimited_joints[4] -= i
     
        elif(key=="y"):
            unlimited_joints[5] += i
        elif(key=="h"):
            unlimited_joints[5] -= i

        elif(key=="z"):
            unlimited_joints = [0, 0, 0, 0, 0, 0]
        elif(key=="m"):
            unlimited_joints = [0.0, 0.78, -1.57, 0, 0.78, 0.0]

        # Limit position
        self.joint_position = self.limit_joints(unlimited_joints)

        # Send position command through MoveIt
        self.move_group.go(self.joint_position)
        self.move_group.stop()


    def limit_joints(self, q):
        limited = copy.copy(q)
        for i in range(6):
            if q[i] > self.joint_upper_limit[i]:
                limited[i] = self.joint_upper_limit[i]
            elif q[i] < self.joint_lower_limit[i]:
                limited[i] = self.joint_lower_limit[i]
        return limited


if __name__ == '__main__':
    print('starting elfin_moveit_keyboard')
    commander = moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('elfin_moveit_keyboard', disable_signals=True)

    node = ElfinMoveItKeyboardNode(commander)

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')
