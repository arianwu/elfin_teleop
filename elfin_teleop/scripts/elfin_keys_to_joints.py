#!/usr/bin/env python
import time
import copy
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# ---------------- CLASS ----------------
class ElfinKeysToJointsNode():
    def __init__(self):
        print('ElfinKeysToJointsNode: initializing node')

        self.ready = False
        self.joint_states = None

        # Initialize variables
        self.joint_upper_limit = [ 3.14,  2.35,  2.61,  3.14,  2.56,  3.14]
        self.joint_lower_limit = [-3.14, -2.35, -2.61, -3.14, -2.56, -3.14]
        self.joint_position = [0, 0, 0, 0, 0, 0]
        self.joint_names = ['elfin_joint1', 'elfin_joint2', 'elfin_joint3', 'elfin_joint4', 'elfin_joint5', 'elfin_joint6']
        self.cmd_joints = JointState()
        self.cmd_joints.name = self.joint_names

        # ROS Infrastructure
        topic = 'keys'
        self.sub_keys = rospy.Subscriber(topic, String, self.keys_callback)
        topic = 'joint_cmd'
        self.pub_joint_cmd = rospy.Publisher(topic, JointState, queue_size=10)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        self.joint_position = list(self.joint_states.position)

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

        self.joint_position = self.limit_joints(unlimited_joints)
        
        self.cmd_joints.header.stamp = rospy.Time.now()
        self.cmd_joints.position = self.joint_position
        self.pub_joint_cmd.publish(self.cmd_joints)


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
    print('starting elfin_keys_to_joints')
    rospy.init_node('elfin_keys_to_joints', disable_signals=True)
    node = ElfinKeysToJointsNode()

    try:
        rospy.spin()
    except:  # rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
