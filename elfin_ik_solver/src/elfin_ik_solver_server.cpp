#include "elfin_ik_solver/elfin_ik_solver_server.h"

ElfinIKSolverServer::ElfinIKSolverServer()
{
	// Initialize Variables
	std::string chain_start = "elfin_base_link";
	std::string chain_end = "elfin_end_link";
	std::string urdf_param = "/robot_description";
	double timeout = 0.005;
	double eps = 1e-5;
	tracik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
	nominal = KDL::JntArray(6);

	// Initialize JointState message
	calculated_response.found.data = false;
	calculated_response.solution.name.resize(ELFIN_DOF_JOINTS);
	calculated_response.solution.position.resize(ELFIN_DOF_JOINTS);
	calculated_response.solution.name = joint_names;

	// Initialize FK solver
	bool valid = tracik_solver->getKDLChain(chain);
	if (!valid)
	{
		ROS_ERROR("There was no valid KDL chain found");
		return;
	}

	// valid = tracik_solver.getKDLLimits(ll, ul);

	// if (!valid)
	// {
	// 	ROS_ERROR("There were no valid KDL joint limits found");
	// 	return;
	// }
	fk_solver = new KDL::ChainFkSolverPos_recursive(chain);

	// ROS Infrastructure
	calculate_ik_srv = nh.advertiseService(CALCULATE_IK_SERVER_TOPIC, &ElfinIKSolverServer::calculateIKCallback, this);
	calculate_fk_srv = nh.advertiseService(CALCULATE_FK_SERVER_TOPIC, &ElfinIKSolverServer::calculateFKCallback, this);
}

ElfinIKSolverServer::~ElfinIKSolverServer()
{
	delete tracik_solver;
	delete fk_solver;
	nh.shutdown();
}

bool ElfinIKSolverServer::calculateIKCallback(elfin_ik_solver_msgs::CalculateIK::Request& request, elfin_ik_solver_msgs::CalculateIK::Response& response)
{
	// Checking for the length of seed state
	if (request.seed_state.size() != ELFIN_DOF_JOINTS){
		ROS_ERROR("SEED STATE HAS INCORRECT LENGTH");
		request.seed_state = {0, 0, 0, 0, 0, 0};
	}
	// No empty quaternions allowed
	geometry_msgs::Quaternion o = request.desired_pose.orientation;
	if (o.x==0 && o.y==0 && o.z==0 && o.w==0){
		ROS_ERROR("EMPTY QUATERNION");
		return false;
	}
	// Normalize quaternion
	double quat_norm = sqrt(pow(o.x, 2) + pow(o.y, 2) + pow(o.z, 2) + pow(o.w, 2));
	o.x /= quat_norm; o.y /= quat_norm; o.z /= quat_norm; o.w /= quat_norm;

	// Retrieve translation
	geometry_msgs::Point pos = request.desired_pose.position;
	trans = KDL::Vector(pos.x, pos.y, pos.z);

	// Retrieve orientation
	orient = KDL::Rotation::Quaternion(o.x, o.y, o.z, o.w);

	// Set seed state
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++)
		nominal(i) = request.seed_state[i];

	// Obtain IK solution
	end_effector_pose = KDL::Frame(orient, trans);
	int result_count = tracik_solver->CartToJnt(nominal, end_effector_pose, result);

	// If none found send false
	if (result_count <= 0) {
		calculated_response.found.data = false;
		calculated_response.solution.position.resize(0);
	}
	// Else send the solution back
	else {
		calculated_response.found.data = true;
		calculated_response.solution.position.resize(ELFIN_DOF_JOINTS);
		for (size_t i=0; i<ELFIN_DOF_JOINTS; i++)
			calculated_response.solution.position[i] = result(i);
		calculated_response.solution.header.stamp = ros::Time::now();
	}
	ROS_INFO("SOLUTIONS FOUND: %d", result_count);

	// Send response
	response = calculated_response;
	return true;
}

bool ElfinIKSolverServer::calculateFKCallback(elfin_ik_solver_msgs::CalculateFK::Request& request, elfin_ik_solver_msgs::CalculateFK::Response& response)
{
	// Initialize variables
	KDL::JntArray joints(6);
	KDL::Frame pose;

	// Extract joints
	for (size_t i=0; i<ELFIN_DOF_JOINTS; i++)
		joints(i) = request.joint_state.position[i];

	// Calculate Forward Kinematics
	fk_solver->JntToCart(joints, pose);

	// Extract translation and orientation from obtained pose
	KDL::Vector trans = pose.p;
	KDL::Rotation orient = pose.M;

	// Set the response
	response.pose.position.x = trans(0);
	response.pose.position.y = trans(1);
	response.pose.position.z = trans(2);
	orient.GetQuaternion(response.pose.orientation.x, 
						 response.pose.orientation.y, 
						 response.pose.orientation.z, 
						 response.pose.orientation.w);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "elfin_ik_solver_server");

	ElfinIKSolverServer node;
	ros::spin();

	return 0;
}