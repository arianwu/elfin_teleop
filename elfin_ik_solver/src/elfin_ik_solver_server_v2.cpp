#include "elfin_ik_solver/elfin_ik_solver_server_v2.h"

ElfinIKSolverServer::ElfinIKSolverServer()
{
	// Initialize JointState message
	calculated_response.found.data = false;
	calculated_response.solution.name.resize(ELFIN_DOF_JOINTS);
	calculated_response.solution.position.resize(ELFIN_DOF_JOINTS);
	calculated_response.solution.name = joint_names;

	// ROS Infrastructure
	calculate_ik_srv = nh.advertiseService(CALCULATE_IK_SERVER_TOPIC, &ElfinIKSolverServer::calculateIKCallback, this);
	calculate_fk_srv = nh.advertiseService(CALCULATE_FK_SERVER_TOPIC, &ElfinIKSolverServer::calculateFKCallback, this);
}

ElfinIKSolverServer::~ElfinIKSolverServer()
{
	delete fastik_solver;
	nh.shutdown();
}

bool ElfinIKSolverServer::calculateIKCallback(elfin_ik_solver_msgs::CalculateIK::Request& request, elfin_ik_solver_msgs::CalculateIK::Response& response)
{
	// Retrieve translation
	geometry_msgs::Point pos = request.desired_pose.position;
	if (abs(pos.x) < 1e-6 && abs(pos.y) < 1e-6)
		pos.x = 1e-6;
	trans = Eigen::Vector3d(pos.x, pos.y, pos.z);

	// Retrieve orientation
	geometry_msgs::Quaternion o = request.desired_pose.orientation;
	Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);
	orient = quat.normalized().toRotationMatrix();
	
	// Set seed state
	seed_state = request.seed_state;

	// Obtain IK solution
	bool found = fastik_solver->getPositionIK(trans, orient, vfree, seed_state, solution);

	// If found send back
	if (found) {
		ROS_INFO("SOLUTION FOUND");
		calculated_response.found.data = true;
		calculated_response.solution.position.resize(ELFIN_DOF_JOINTS);
		calculated_response.solution.position = solution;
		calculated_response.solution.header.stamp = ros::Time::now();
	}
	else {
		ROS_WARN("NO SOLUTION FOUND");
		calculated_response.found.data = false;
		calculated_response.solution.position.resize(0);
	}

	// Send response
	response = calculated_response;
	return true;
}

bool ElfinIKSolverServer::calculateFKCallback(elfin_ik_solver_msgs::CalculateFK::Request& request, elfin_ik_solver_msgs::CalculateFK::Response& response)
{
	// Initialize variables
	std::vector<double> joint_angles = request.joint_state.position;
	Eigen::Vector3d trans;
	Eigen::Matrix3d orient;

	// Calculate Forward Kinematics
	fastik_solver->getPositionFK(joint_angles, trans, orient);

	// Extract translation and orientation from obtained pose
	Eigen::Quaterniond quat(orient);

	// Set the response
	response.pose.position.x = trans(0);
	response.pose.position.y = trans(1);
	response.pose.position.z = trans(2);
	
	response.pose.orientation.w = quat.w();
	response.pose.orientation.x = quat.x();
	response.pose.orientation.y = quat.y();
	response.pose.orientation.z = quat.z(); 

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "elfin_ik_solver_server");

	ElfinIKSolverServer node;
	ros::spin();

	return 0;
}