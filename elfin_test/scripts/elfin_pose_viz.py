#!/usr/bin/env python
import rospy
import numpy as np

import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState


# ---------------- CLASS ----------------
class ElfinPoseVisualizerNode():
    def __init__(self):
        print('ElfinPoseVisualizerNode: initializing node')

        self.frame_id = "elfin_base_link"

        # ROS Infrastructure
        topic = "pose_cmd"
        self.pose_cmd_sub = rospy.Subscriber(topic, Pose, self.pose_cmd_callback)
        topic = topic + "_stamped"
        self.pose_cmd_pub = rospy.Publisher(topic, PoseStamped, queue_size=1)

        topic = "joint_states"
        self.joint_states_sub = rospy.Subscriber(topic, JointState, self.joint_states_callback)
        topic = topic + "_stamped"
        self.joint_states_pub = rospy.Publisher(topic, PoseStamped, queue_size=1)


    def pose_cmd_callback(self, msg):
        send_msg = PoseStamped()
        send_msg.header.frame_id = self.frame_id
        send_msg.pose = msg
        self.pose_cmd_pub.publish(send_msg)


    def joint_states_callback(self, msg):
        send_msg = PoseStamped()
        position, quaternion = self.fkine(msg.position)
        send_msg.header.frame_id = self.frame_id
        send_msg.pose.position = position
        send_msg.pose.orientation = quaternion
        self.joint_states_pub.publish(send_msg)


    def dh(self, th, d, alpha, a):
        mat = np.array([[np.cos(th), -np.cos(alpha)*np.sin(th),  np.sin(alpha)*np.sin(th), a*np.cos(th)],
                        [np.sin(th),  np.cos(alpha)*np.cos(th), -np.sin(alpha)*np.cos(th), a*np.sin(th)],
                        [         0,             np.sin(alpha),             np.cos(alpha),            d],
                        [         0,                         0,                         0,            1]])
        return mat


    def fkine(self, q):
        T1 = self.dh(q[0]          , 0.22,  np.pi/2, 0   )
        T2 = self.dh(q[1] + np.pi/2, 0   ,  np.pi  , 0.38)
        T3 = self.dh(q[2] + np.pi/2, 0   ,  np.pi/2, 0   )
        T4 = self.dh(q[3] + np.pi  , 0.42,  np.pi/2, 0   )
        T5 = self.dh(q[4]          , 0   , -np.pi/2, 0   )
        T6 = self.dh(q[5] + np.pi  , 0.18,  0      , 0   )
        pose = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)

        trans = tf.transformations.translation_from_matrix(pose)
        orient = tf.transformations.quaternion_from_matrix(pose)
        return Point(trans[0], trans[1], trans[2]), Quaternion(orient[0], orient[1], orient[2], orient[3])


if __name__ == '__main__':
    print('starting elfin_pose_visualizer')
    rospy.init_node('elfin_pose_visualizer', disable_signals=True)
    node = ElfinPoseVisualizerNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')
