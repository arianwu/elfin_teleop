#include "elfin_ik_solver/elfin_ik_solver_node.h"

ElfinIKSolverNode::ElfinIKSolverNode()
{
	// Initialize Variables
	std::string chain_start = "elfin_base";
	std::string chain_end = "elfin_end_link";
	std::string urdf_param = "/robot_description";
	double timeout = 0.005;
	double eps = 1e-5;
	tracik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
	nominal = KDL::JntArray(6);

	// Initialize JointState message
	desired_joint_state.name.resize(ELFIN_DOF_JOINTS);
	desired_joint_state.position.resize(ELFIN_DOF_JOINTS);
	desired_joint_state.name = joint_names;

	// ROS Infrastructure
	desired_joint_state_pub = nh.advertise<sensor_msgs::JointState>(DESIRED_JOINT_STATE_TOPIC, 5);
	commanded_pose_sub = nh.subscribe(COMMANDED_POSE_TOPIC, 1, &ElfinIKSolverNode::poseCallback, this);
}

ElfinIKSolverNode::~ElfinIKSolverNode()
{
	nh.shutdown();
}

void ElfinIKSolverNode::poseCallback(const geometry_msgs::Pose& msg)
{	
	// Retrieve translation
	geometry_msgs::Point pos = msg.position;
	trans = KDL::Vector(pos.x, pos.y, pos.z);

	// Retrieve orientation
	geometry_msgs::Quaternion o = msg.orientation;
	orient = KDL::Rotation::Quaternion(o.x, o.y, o.z, o.w);

	// Obtain IK solution
	end_effector_pose = KDL::Frame(orient, trans);
	int result_count = tracik_solver->CartToJnt(nominal, end_effector_pose, result);

	ROS_INFO("SOLUTIONS FOUND: %d", result_count);
	if (result_count < 0) {
		ROS_WARN("SOLUTION NOT FOUND, IGNORING COMMANDED POSE");
		return;
	}

	for(size_t i=0; i<ELFIN_DOF_JOINTS; i++)
		desired_joint_state.position[i] = result(i);
	desired_joint_state.header.stamp = ros::Time::now();
	desired_joint_state_pub.publish(desired_joint_state);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "elfin_ik_solver_node");

	ElfinIKSolverNode node;
	ros::spin();

	return 0;
}