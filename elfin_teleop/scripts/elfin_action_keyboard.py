#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *

# ---------------- CLASS ----------------
class ElfinActionKeyboardNode():
    def __init__(self):
        print('ElfinActionKeyboardNode: initializing node')

        self.ready = False
        self.joint_states = None

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
        self.exec_time = 1

        # ROS Infrastructure
        topic = 'keys'
        self.sub_joint_cmd = rospy.Subscriber(topic, String, self.keys_callback)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)
        
        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        self.joint_position = list(self.joint_states.position)
        
        # Action Library for ros_control
        namespace = 'elfin_arm_controller'
        self.robot_client = actionlib.SimpleActionClient(namespace + '/follow_joint_trajectory', FollowJointTrajectoryAction)

        print('Waiting for server...')
        self.robot_client.wait_for_server()
        print('Connected to server')

        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.joint_names

        # Initial Position (Optional)
        #self.joint_position = self.limit_joints(self.joint_position)
        #self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(2.0))]
        #self.robot_client.send_goal(self.g)
        #self.robot_client.wait_for_result()
        rospy.sleep(1)


    def keys_callback(self, msg):
        if not self.ready:
            return

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
            unlimited_joints = [1.57, 0.78, -1.57, 0, 0.78, 0.0]
        elif(key=="x"):
            unlimited_joints = list(self.joint_states.position)

        # Limit position
        self.joint_position = self.limit_joints(unlimited_joints)

        # Send goal
        self.robot_client.cancel_goal()
        self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(self.exec_time))]
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
    print('starting elfin_action_keyboard')
    rospy.init_node('elfin_action_keyboard', disable_signals=True)
    node = ElfinActionKeyboardNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')