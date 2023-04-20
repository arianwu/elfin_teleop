#!/usr/bin/env python
import time
import copy
import roslib
import rospy
import tf.transformations
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion

# ---------------- CLASS ----------------
class ElfinKeyToPoseNode():
    def __init__(self):
        print('ElfinKeyToPoseNode: initializing node')

        self.ready = False
        self.joint_states = None

        # ROS Infrastructure
        topic = 'keys'
        self.sub_keys = rospy.Subscriber(topic, String, self.keys_callback)
        topic = 'pose_cmd'
        self.pub_pose_cmd = rospy.Publisher(topic, Pose, queue_size=10)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')
        
        self.desired_position, self.desired_orientation = self.fkine(self.joint_states.position)
        self.desired_pose = Pose(self.desired_position, self.desired_orientation)

        rospy.sleep(1)


    def keys_callback(self, msg):
        if not self.ready:
            return

        i = 0.1
        key = msg.data

        # Position
        if(key=="q"):
            self.desired_position.x += i
        elif(key=="a"):
            self.desired_position.x -= i
        
        elif(key=="w"):
            self.desired_position.y += i
        elif(key=="s"):
            self.desired_position.y -= i
     
        elif(key=="e"):
            self.desired_position.z += i
        elif(key=="d"):
            self.desired_position.z -= i

        # Rotation wrt base
        elif(key=="r"):
            rot = tf.transformations.quaternion_about_axis(i, (1, 0, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="f"):
            rot = tf.transformations.quaternion_about_axis(-i, (1, 0, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
     
        elif(key=="t"):
            rot = tf.transformations.quaternion_about_axis(i, (0, 1, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="g"):
            rot = tf.transformations.quaternion_about_axis(-i, (0, 1, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
     
        elif(key=="y"):
            rot = tf.transformations.quaternion_about_axis(i, (0, 0, 1))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="h"):
            rot = tf.transformations.quaternion_about_axis(-i, (0, 0, 1))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply(rot, [o.x, o.y, o.z, o.w])
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])

        # Rotation wrt ee
        elif(key=="u"):
            rot = tf.transformations.quaternion_about_axis(i, (1, 0, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="j"):
            rot = tf.transformations.quaternion_about_axis(-i, (1, 0, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
     
        elif(key=="i"):
            rot = tf.transformations.quaternion_about_axis(i, (0, 1, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="k"):
            rot = tf.transformations.quaternion_about_axis(-i, (0, 1, 0))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
     
        elif(key=="o"):
            rot = tf.transformations.quaternion_about_axis(i, (0, 0, 1))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])
        elif(key=="l"):
            rot = tf.transformations.quaternion_about_axis(-i, (0, 0, 1))
            o = self.desired_orientation
            result = tf.transformations.quaternion_multiply([o.x, o.y, o.z, o.w], rot)
            self.desired_orientation = Quaternion(result[0], result[1], result[2], result[3])

        elif(key=="z"):
            self.desired_position, self.desired_orientation = self.fkine([0, 0, 0, 0, 0, 0])
        elif(key=="x"):
            self.desired_position, self.desired_orientation = self.fkine(self.joint_states.position)
        elif(key=="m"):
            self.desired_position, self.desired_orientation = self.fkine([1.57, 0.78, -1.57, 0, 0.78, 0.0])

        self.desired_pose.position = self.desired_position
        self.desired_pose.orientation = self.desired_orientation

        self.pub_pose_cmd.publish(self.desired_pose)


    def joint_states_callback(self, msg):
        self.joint_states = msg
        self.ready = True


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
    print('starting elfin_keys_to_pose')
    rospy.init_node('elfin_keys_to_pose', disable_signals=True)
    node = ElfinKeyToPoseNode()

    try:
        rospy.spin()
    except:  # rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
