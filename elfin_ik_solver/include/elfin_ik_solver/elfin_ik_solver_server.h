#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "elfin_ik_solver_msgs/CalculateIK.h"
#include "elfin_ik_solver_msgs/CalculateFK.h"

enum eJointName
{
	ELFIN_JOINT_1,
	ELFIN_JOINT_2,
	ELFIN_JOINT_3,
	ELFIN_JOINT_4,
	ELFIN_JOINT_5,
	ELFIN_JOINT_6,
	ELFIN_DOF_JOINTS
};

const std::string COMMANDED_POSE_TOPIC = "pose_cmd";
const std::string JOINT_STATE_TOPIC = "joint_state";
const std::string DESIRED_JOINT_STATE_TOPIC = "joint_cmd";
const std::string CALCULATE_IK_SERVER_TOPIC = "calculate_ik";
const std::string CALCULATE_FK_SERVER_TOPIC = "calculate_fk";

class ElfinIKSolverServer {
public:
	ElfinIKSolverServer();
	~ElfinIKSolverServer();

	bool calculateIKCallback(elfin_ik_solver_msgs::CalculateIK::Request& request, elfin_ik_solver_msgs::CalculateIK::Response& response);

	bool calculateFKCallback(elfin_ik_solver_msgs::CalculateFK::Request& request, elfin_ik_solver_msgs::CalculateFK::Response& response);

private:
	ros::NodeHandle nh;
	ros::ServiceServer calculate_ik_srv;
	ros::ServiceServer calculate_fk_srv;

	elfin_ik_solver_msgs::CalculateIK::Response calculated_response;

	std::vector<std::string> joint_names = {
		"elfin_joint1",
		"elfin_joint2",
		"elfin_joint3",
		"elfin_joint4",
		"elfin_joint5",
		"elfin_joint6"
	};

	// Variables for FK
	KDL::Chain chain;
	KDL::ChainFkSolverPos_recursive* fk_solver;// = KDL::ChainFkSolverPos_recursive(chain);

	// Variables for IK
	TRAC_IK::TRAC_IK* tracik_solver;
	KDL::JntArray nominal;
	KDL::Vector trans;
	KDL::Rotation orient;
	KDL::Frame end_effector_pose;
	KDL::JntArray result;
};
