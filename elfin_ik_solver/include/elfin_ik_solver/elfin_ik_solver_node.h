#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <trac_ik/trac_ik.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

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

class ElfinIKSolverNode {
public:
	ElfinIKSolverNode();
	~ElfinIKSolverNode();

	void poseCallback(const geometry_msgs::Pose& msg);
	void jointStateCallback(const sensor_msgs::JointState& msg);
private:
	ros::NodeHandle nh;

	ros::Publisher desired_joint_state_pub;
	ros::Subscriber commanded_pose_sub;

	sensor_msgs::JointState desired_joint_state;

	std::vector<std::string> joint_names = {
		"elfin_joint1",
		"elfin_joint2",
		"elfin_joint3",
		"elfin_joint4",
		"elfin_joint5",
		"elfin_joint6"
	};

	TRAC_IK::TRAC_IK* tracik_solver;
	KDL::JntArray nominal;
	KDL::Vector trans;
	KDL::Rotation orient;
	KDL::Frame end_effector_pose;
	KDL::JntArray result;
};
