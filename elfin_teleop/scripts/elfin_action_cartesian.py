#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from elfin_ik_solver_msgs.srv import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
import tf.transformations as transform
import numpy as np
from rospy.numpy_msg import numpy_msg

# ----------------- TODO ----------------

# Add security measure to modify path planning time if movement is to big or if singularity is near
# Warn when changing the time of path planning
# Calculate time to achieve desired position, if greater than 


# ---------------- CLASS ----------------
class ElfinActionCartesianNode():
    def __init__(self):
        print('ElfinActionCartesianNode: initializing node')
        
        # Initialize Variables
        self.ready = False

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
        self.joint_velocity_limit = np.array([1.57, 1.57, 1.57, 1.57, 1.57, 1.57])*0.25
        self.joint_position_threshold = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.exec_time = 1

        self.current_joint_states = JointState()

        # ROS Infrastructure
        topic = 'joint_states'
        self.joint_states_sub = rospy.Subscriber(topic, numpy_msg(JointState), self.joint_states_callback)
        topic = 'pose_cmd'
        self.sub_pose_cmd = rospy.Subscriber(topic, numpy_msg(Pose), self.pose_cmd_callback)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        # Things for interpolation
        self.previous_pose_cmd = self.calculate_fk_client(self.current_joint_states)
        self.previous_joints = self.current_joint_states.position

        # Service thingies
        print('Waiting for calculate_ik and calculate_fk...')
        rospy.wait_for_service('calculate_ik')
        rospy.wait_for_service('calculate_fk')
        print('Connected to calculate_ik and calculate_fk')

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


    def pose_cmd_callback(self, msg):
        if not self.ready:
            return

        current_position = np.array([msg.position.x, msg.position.y, msg.position.z])
        current_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        previous_position = np.array([self.previous_pose_cmd.position.x, self.previous_pose_cmd.position.y, self.previous_pose_cmd.position.z])
        previous_orientation = np.array([self.previous_pose_cmd.orientation.x, self.previous_pose_cmd.orientation.y, self.previous_pose_cmd.orientation.z, self.previous_pose_cmd.orientation.w])
        
        total_points = 5
        total_time = self.exec_time
        points = []
        actual_time = 0
        time_step = 1 / float(total_points - 1)

        for i in range(total_points):
            s = i / float(total_points - 1)
            calculated_position = (current_position - previous_position) * s + previous_position
            calculated_orientation = transform.quaternion_slerp(previous_orientation, current_orientation, s)
            calculated_pose = Pose()
            calculated_pose.position.x = calculated_position[0]
            calculated_pose.position.y = calculated_position[1]
            calculated_pose.position.z = calculated_position[2]
            calculated_pose.orientation.x = calculated_orientation[0]
            calculated_pose.orientation.y = calculated_orientation[1]
            calculated_pose.orientation.z = calculated_orientation[2]
            calculated_pose.orientation.w = calculated_orientation[3]
            solution = self.calculate_ik_client(calculated_pose, self.previous_joints)
            
            # SECURITY STUFF TO LIMIT VELOCITY IF GOAL IS TOO FAR AWAY (avoids big movements at great speed)
            # Calculate distance between solution and previous joints, If greater than threshold limit joint velocity
            if (np.abs(np.array(solution.position) - self.previous_joints) > self.joint_position_threshold).any():
                # Calculate the times
                times = np.abs(np.array(solution.position) - self.previous_joints) / self.joint_velocity_limit

                # If greater than execution time, modify the execution time 
                if (times > time_step).any():
                    actual_time += times.max() - time_step
                    rospy.logwarn("EXECUTION TIME FOR POINT %d ADJUSTED FROM %.2f TO %.2f", i+1, time_step, times.max())
            # END SECURITY STUFF

            actual_time += time_step

            if solution != None and len(solution.position) > 0:
                self.previous_joints = solution.position
                point = JointTrajectoryPoint(positions=solution.position, velocities=solution.velocity, time_from_start=rospy.Duration(actual_time))
            else:
                point = JointTrajectoryPoint(positions=self.previous_joints, velocities=solution.velocity, time_from_start=rospy.Duration(actual_time))
                
            points.append(point)

        # Update previous pose
        self.previous_pose_cmd = msg

        # Send goal
        self.robot_client.cancel_goal()
        self.g.trajectory.points = points
        self.robot_client.send_goal(self.g)
        self.robot_client.wait_for_result()
        ROS_INFO("TRAJECTORY DONE")


    def joint_states_callback(self, msg):
        self.current_joint_states = msg
        self.ready = True


    def calculate_ik_client(self, pose, seed_state):
        try:
            calculate_ik = rospy.ServiceProxy('calculate_ik', CalculateIK)
            response = calculate_ik(pose, seed_state)
            if response.found.data:
                return response.solution
            else:
                return JointState()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def calculate_fk_client(self, joint_state):
        try:
            calculate_fk = rospy.ServiceProxy('calculate_fk', CalculateFK)
            response = calculate_fk(joint_state)
            return response.pose

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == '__main__':
    print('starting elfin_action_cartesian')
    rospy.init_node('elfin_action_cartesian', disable_signals=True)
    node = ElfinActionCartesianNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')