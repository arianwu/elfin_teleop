#include "elfin_teleop/elfin_keys_to_joints.h"

ElfinKeysToJointNode::ElfinKeysToJointNode(){
	// Initialize Variables
	ready = false;
	joint_state_cmd.name = joint_names;

	// ROS Infrastructure
	joint_cmd_pub = nh.advertise<sensor_msgs::JointState>(JOINT_CMD_TOPIC, 5);
	keys_sub = nh.subscribe(KEYS_TOPIC, 1, &ElfinKeysToJointNode::keysCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &ElfinKeysToJointNode::jointStateCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Initialize position
	joint_position = current_joint_state.position;
}

ElfinKeysToJointNode::~ElfinKeysToJointNode(){
	nh.shutdown();
}

void ElfinKeysToJointNode::keysCallback(const std_msgs::String::ConstPtr& msg){
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

	// Limit position
	limitJoints();

	// Send message
	joint_state_cmd.header.stamp = ros::Time::now();
	joint_state_cmd.position = joint_position;
	joint_cmd_pub.publish(joint_state_cmd);
}

void ElfinKeysToJointNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void ElfinKeysToJointNode::limitJoints(){
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "elfin_keys_to_joints");

	ElfinKeysToJointNode node;
	ros::spin();

	return 0;
}