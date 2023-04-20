#include "elfin_teleop/elfin_action.h"

ElfinActionNode::ElfinActionNode(){
	// Initialize Variables
	exec_time = 0.1;
	first_read = true;
	ready = false;
	times.resize(6);

	// ROS Infrastructure
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 1, &ElfinActionNode::jointCmdCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &ElfinActionNode::jointStateCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Action Library for ros_control
	robot_client = new Client("elfin_arm_controller/follow_joint_trajectory", true);

	ROS_INFO("Waiting for server...");
	robot_client->waitForServer();
	ROS_INFO("Connected to server");

	goal.trajectory.joint_names = joint_names;
	goal.trajectory.points.resize(1);
	goal.trajectory.points[0].positions.resize(6);
	goal.trajectory.points[0].velocities.resize(6);
	goal.trajectory.points[0].velocities = joint_velocity;
}

ElfinActionNode::~ElfinActionNode(){
	delete robot_client;
	nh.shutdown();
}

void ElfinActionNode::jointCmdCallback(const sensor_msgs::JointState& msg){
	// Extract data from message
	joint_position = msg.position;

	// Limit position
	limitJoints();

	if (first_read){
		previous_joint_position = current_joint_state.position;
		first_read = false;
	}


	// SECURITY STUFF TO LIMIT VELOCITY IF GOAL IS TOO FAR AWAY (avoids big movements at great speed)
	double exec_time;
	{
		// Set initial execution time
		exec_time = this-> exec_time;
		bool limit = false;
		for(size_t i=0; i<ELFIN_DOF_JOINTS; i++){
			if (abs(joint_position[i] - previous_joint_position[i]) > joint_position_threshold[i])
				limit = true;
		}

		if (limit || false) {
			// Calculate the times required to reach goal using max velocity
			for (size_t i=0; i<ELFIN_DOF_JOINTS; i++)
				times[i] = abs(joint_position[i] - previous_joint_position[i])/(joint_velocity_limit[i] * velocity_scale);
		
			// If max time required is greater than actual execution time, change execution time
			if (*max_element(times.begin(), times.end()) > exec_time){
				exec_time = *max_element(times.begin(), times.end());
				ROS_WARN("EXECUTION TIME ADJUSTED TO %.2f", exec_time);
			}
		}
	} // END SECURITY STUFF

	// Update previous joint position
	previous_joint_position = joint_position;

	// Send goal
	goal.trajectory.points[0].positions = joint_position;
	goal.trajectory.points[0].time_from_start = ros::Duration(exec_time);
	goal.trajectory.header.stamp = ros::Time::now();

	robot_client->sendGoal(goal);
	robot_client->waitForResult();
}

void ElfinActionNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void ElfinActionNode::limitJoints(){
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "elfin_action");

	ElfinActionNode node;
	ros::spin();

	return 0;
}