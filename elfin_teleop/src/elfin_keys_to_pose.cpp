#include "elfin_teleop/elfin_keys_to_pose.h"

ElfinKeysToPoseNode::ElfinKeysToPoseNode(){
	// Initialize Variables
	ready = false;

	// ROS Infrastructure
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &ElfinKeysToPoseNode::jointStateCallback, this);
	pose_cmd_pub = nh.advertise<geometry_msgs::Pose>(POSE_CMD_TOPIC, 5);
	keys_sub = nh.subscribe(KEYS_TOPIC, 1, &ElfinKeysToPoseNode::keysCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");
	
	// Initialize desire position, orientation and pose	
	forwardKinematics(current_joint_state.position, desired_position, desired_orientation);
	desired_pose.position = desired_position;
	desired_pose.orientation = desired_orientation;
}

ElfinKeysToPoseNode::~ElfinKeysToPoseNode(){
	nh.shutdown();
}

void ElfinKeysToPoseNode::keysCallback(const std_msgs::String::ConstPtr& msg){
	if (!ready)
		return;

	double i = 0.05;
	tf2::Vector3 axis;
	tf2::Quaternion rot, current_quat, result;
	std::vector<double> zero_vector, main_vector = {1.57, 0.78, -1.57, 0, 0.78, 0.0};
	zero_vector.resize(6);

	std::string key = msg->data.c_str();
	char key_char = (char) key[0];
	switch (key_char) {
	// Position
	case 'q':
		desired_position.x += i;
		break;
	case 'a':
		desired_position.x -= i;
		break;
	case 'w':
		desired_position.y += i;
		break;
	case 's':
		desired_position.y -= i;
		break;
	case 'e':
		desired_position.z += i;
		break;
	case 'd':
		desired_position.z -= i;
		break;
	// Rotation wrt base
	case 'r':
		axis = tf2::Vector3(1, 0, 0);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'f':
		axis = tf2::Vector3(1, 0, 0);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 't':
		axis = tf2::Vector3(0, 1, 0);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'g':
		axis = tf2::Vector3(0, 1, 0);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'y':
		axis = tf2::Vector3(0, 0, 1);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'h':
		axis = tf2::Vector3(0, 0, 1);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = rot * current_quat;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	// Rotation wrt End Efector
	case 'u':
		axis = tf2::Vector3(1, 0, 0);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'j':
		axis = tf2::Vector3(1, 0, 0);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'i':
		axis = tf2::Vector3(0, 1, 0);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'k':
		axis = tf2::Vector3(0, 1, 0);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'o':
		axis = tf2::Vector3(0, 0, 1);
		rot = tf2::Quaternion(axis, i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	case 'l':
		axis = tf2::Vector3(0, 0, 1);
		rot = tf2::Quaternion(axis, -i);
		current_quat = tf2::Quaternion(desired_orientation.x, desired_orientation.y, desired_orientation.z, desired_orientation.w);
		result = current_quat * rot;
		result.normalize();
		desired_orientation.x = result.x();
		desired_orientation.y = result.y();
		desired_orientation.z = result.z();
		desired_orientation.w = result.w();
		break;
	// Main and zero positions
	case 'z':
		forwardKinematics(zero_vector, desired_position, desired_orientation);
		break;
	case 'x':
		forwardKinematics(current_joint_state.position, desired_position, desired_orientation);
		break;
	case 'm':
		forwardKinematics(main_vector, desired_position, desired_orientation);
		break;
	}

	desired_pose.position = desired_position;
	desired_pose.orientation = desired_orientation;

	pose_cmd_pub.publish(desired_pose);
}

void ElfinKeysToPoseNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void ElfinKeysToPoseNode::denavitHartenberg(double th, double d, double alpha, double a, Eigen::Matrix4d &dh){
	dh<<cos(th), -cos(alpha)*sin(th),  sin(alpha)*sin(th), a*cos(th),
    	sin(th),  cos(alpha)*cos(th), -sin(alpha)*cos(th), a*sin(th),
    	      0,          sin(alpha),          cos(alpha),         d,
    	      0,                   0,                   0,         1;
}

void ElfinKeysToPoseNode::forwardKinematics(std::vector<double> &q, geometry_msgs::Point &trans, geometry_msgs::Quaternion &orient){
	// Initialize Variables for calculation
	Eigen::Matrix4d T1, T2, T3, T4, T5, T6, fkine;
	Eigen::Matrix3d orient_eigen;
	Eigen::Vector3d trans_eigen;
	Eigen::Quaterniond quat;

	// Obtain Pose of End Efector
	denavitHartenberg(q[0]         , 0.22,  M_PI_2, 0   , T1);
	denavitHartenberg(q[1] + M_PI_2, 0   ,  M_PI  , 0.38, T2);
	denavitHartenberg(q[2] + M_PI_2, 0   ,  M_PI_2, 0   , T3);
	denavitHartenberg(q[3] + M_PI  , 0.42,  M_PI_2, 0   , T4);
	denavitHartenberg(q[4]         , 0   , -M_PI_2, 0   , T5);
	denavitHartenberg(q[5] + M_PI  , 0.18,  0     , 0   , T6);
	fkine = T1 * T2 * T3 * T4 * T5 * T6;

	// Extract position and orientation
	trans_eigen<< fkine(0,3), fkine(1,3), fkine(2,3);
	orient_eigen<<	fkine(0,0), fkine(0,1), fkine(0,2),
					fkine(1,0), fkine(1,1), fkine(1,2), 
					fkine(2,0), fkine(2,1), fkine(2,2);
	quat = Eigen::Quaterniond(orient_eigen);

	// Return msgs
	trans.x = trans_eigen(0);
	trans.y = trans_eigen(1);
	trans.z = trans_eigen(2);
	orient.x = quat.x();
	orient.y = quat.y();
	orient.z = quat.z();
	orient.w = quat.w();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "elfin_keys_to_pose");

	ElfinKeysToPoseNode node;
	ros::spin();

	return 0;
}