#include "elfin_teleop/elfin_action_keyboard.h"

ElfinActionKeyboardNode::ElfinActionKeyboardNode(){
	// Initialize Variables
	ready = false;
	exec_time = 1;

	// ROS Infrastructure
	keys_sub = nh.subscribe(KEYS_TOPIC, 1, &ElfinActionKeyboardNode::keysCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &ElfinActionKeyboardNode::jointStateCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Initialize position
	joint_position = current_joint_state.position;

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

ElfinActionKeyboardNode::~ElfinActionKeyboardNode(){
	delete robot_client;
	nh.shutdown();
}

void ElfinActionKeyboardNode::keysCallback(const std_msgs::String::ConstPtr& msg){
	if (!ready)
		return;

	double i = 0.1;
	std::string key = msg->data.c_str();
	char key_char = (char) key[0];
	switch (key_char) {
	case 'q':
		joint_position[0] += i;
		break;
	case 'a':
		joint_position[0] -= i;
		break;
	case 'w':
		joint_position[1] += i;
		break;
	case 's':
		joint_position[1] -= i;
		break;
	case 'e':
		joint_position[2] += i;
		break;
	case 'd':
		joint_position[2] -= i;
		break;
	case 'r':
		joint_position[3] += i;
		break;
	case 'f':
		joint_position[3] -= i;
		break;
	case 't':
		joint_position[4] += i;
		break;
	case 'g':
		joint_position[4] -= i;
		break;
	case 'y':
		joint_position[5] += i;
		break;
	case 'h':
		joint_position[5] -= i;
		break;
	case 'z':
		joint_position = {0, 0, 0, 0, 0, 0};
		break;
	case 'm':
		joint_position = {1.57, 0.78, -1.57, 0, 0.78, 0.0};
	}

	limitJoints();

	//robot_client->cancelGoal();
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++) 
		goal.trajectory.points[0].positions[i] = joint_position[i];
	goal.trajectory.points[0].time_from_start = ros::Duration(exec_time);
	goal.trajectory.header.stamp = ros::Time::now();

	robot_client->sendGoal(goal);
	robot_client->waitForResult();

    // self.robot_client.cancel_goal()
    // self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(0.1))]
    // self.robot_client.send_goal(self.g)
    // self.robot_client.wait_for_result()	
}

void ElfinActionKeyboardNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void ElfinActionKeyboardNode::limitJoints(){
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "elfin_action_keyboard");

	ElfinActionKeyboardNode node;
	ros::spin();

	return 0;
}