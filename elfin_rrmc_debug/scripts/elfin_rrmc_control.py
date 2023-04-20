#!/usr/bin/env python
import rospy
import numpy as np

import tf
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rospy.numpy_msg import numpy_msg

# ---------------- CLASS ----------------
class ElfinRRMCControlNode():
    def __init__(self):
        print('ElfinRRMCControlNode: initializing node')

        # Initialize Variables
        self.first_joint_state = False
        self.first_pose_cmd = False

        self.current_joint_states = None
        self.current_pose_cmd = None
        self.command_message = Float64MultiArray()

        self.joint_upper_limit = np.array([ 3.14,  2.35,  2.61,  3.14,  2.56,  3.14])
        self.joint_lower_limit = np.array([-3.14, -2.35, -2.61, -3.14, -2.56, -3.14])
        self.joint_middle_value = np.array([0, 0, 0, 0, 0, 0.0])

        # ROS Infrastructure
        topic = 'joint_states'
        self.joint_states_sub = rospy.Subscriber(topic, numpy_msg(JointState), self.joint_states_callback)

        topic = 'pose_cmd'
        self.pose_cmd_sub = rospy.Subscriber(topic, numpy_msg(Pose), self.pose_cmd_callback)

        topic = 'elfin_arm_controller/command'
        self.command_pub = rospy.Publisher(topic, numpy_msg(Float64MultiArray), queue_size=1)

        ## PLAYGROUND
        #print np.round(self.jacobian(np.array([0.0, 0, 0, 0, 0, 0])), 2)
        #print()

        # Wait to receive both messages
        print('Waiting for messages...')
        while (not (self.first_joint_state and self.first_pose_cmd)):
            pass
        print('Ready')

        q_cmd = self.current_joint_states.position

        # Main loop
        freq = 200
        dt = 1.0/freq
        self.rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            # Get Joint States
            q = self.current_joint_states.position

            # Get actual X
            x = self.TF2xyzquat(self.fkine(q))

            # Get command
            pos_des = self.current_pose_cmd.position
            quat_des = self.current_pose_cmd.orientation
            xd = np.array([pos_des.x, pos_des.y, pos_des.z, quat_des.w, quat_des.x, quat_des.y, quat_des.z])

            # Calculate error
            e = self.error_calculation(x, xd)

            # Calculate reference d_error
            k = 50
            de = -k * e

            # Calculate Jacobian
            J = self.jacobian(q)

            # Calculate velocities
            print((np.eye(6) - np.linalg.pinv(J).dot(J)).round(9))
            dq0 = self.second_objective(10, q)
            dq = np.linalg.pinv(J).dot(de) + (np.eye(6) - np.linalg.pinv(J).dot(J)).dot(dq0)

            # Integrate
            q_cmd = q + dt*dq

            #print np.round(dq, 2)

            # Send Commands
            self.command_message.data = q_cmd
            self.command_pub.publish(self.command_message)

            self.rate.sleep()


    def joint_states_callback(self, msg):
        self.first_joint_state = True
        self.current_joint_states = msg


    def pose_cmd_callback(self, msg):
        self.first_pose_cmd = True
        self.current_pose_cmd = msg


    def rot2quat(self, R):
        dEpsilon = 1e-6
        quat = 4*[0.,]

        quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
        if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
            quat[1] = 0.0
        else:
            quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
        if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
            quat[2] = 0.0
        else:
            quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
        if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
            quat[3] = 0.0
        else:
            quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

        return np.array(quat)


    def TF2xyzquat(self, T):
        quat = self.rot2quat(T[0:3,0:3])
        res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
        return np.array(res)


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
        return pose

        trans = tf.transformations.translation_from_matrix(pose)
        orient = tf.transformations.quaternion_from_matrix(pose)
        return Point(trans[0], trans[1], trans[2]), Quaternion(orient[0], orient[1], orient[2], orient[3])


    def jacobian(self, q, delta=0.01):
        x0 = self.TF2xyzquat(self.fkine(q))
        J = np.zeros((7, 6))
        for i in range(6):
            deltaq = np.copy(q)
            deltaq[i] += delta

            x_delta = self.TF2xyzquat(self.fkine(deltaq)) - x0

            J[:, i] = x_delta.transpose()
        return J/delta


    def error_calculation(self, x, xd):
        x_pos = x[0:3]
        xd_pos = xd[0:3]
        e_pos = x_pos-xd_pos

        quat2tf = lambda q: np.array([q[1], q[2], q[3], q[0]])
        x_quat = quat2tf(x[3:7])
        xd_quat = quat2tf(xd[3:7])
        e_quat = tf.transformations.quaternion_multiply(x_quat, tf.transformations.quaternion_inverse(xd_quat)) - np.array([0,0,0,1])

        e = np.hstack([e_pos, e_quat[3], e_quat[0:3]])
        return e


    def second_objective(self, k, q, delta= 0.001):
        dq0 = np.zeros(6)
        w0 = self.w_function(q)

        for i in range(6):
            deltaq = np.copy(q)
            deltaq[i] += delta

            w_delta = self.w_function(deltaq) - w0

            dq0[i] = w_delta
        return k * dq0 / delta

    def w_function(self, q):
        n = 6
        sigma = np.sum(((q - self.joint_middle_value) / (self.joint_upper_limit - self.joint_lower_limit))**2)
        w = -1/2/n * sigma
        return w


if __name__ == '__main__':
    print('starting elfin_rrmc_control')
    rospy.init_node('elfin_rrmc_control', disable_signals=True)
    node = ElfinRRMCControlNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')