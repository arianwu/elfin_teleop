#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include "elfin_ik_solver_msgs/CalculateFK.h"
#include "elfin_ik_solver_msgs/CalculateIK.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

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

const std::string POSE_CMD_TOPIC = "pose_cmd";
const std::string JOINT_CMD_TOPIC = "joint_cmd";
const std::string JOINT_STATE_TOPIC = "joint_states";

const std::string CALCULATE_FK_SERVER_TOPIC = "calculate_fk";
const std::string CALCULATE_IK_SERVER_TOPIC = "calculate_ik";

class ElfinActionCartesianNode {
public:
	ElfinActionCartesianNode();
	~ElfinActionCartesianNode();

	void poseCmdCallback(const geometry_msgs::Pose& msg);
	void jointCmdCallback(const sensor_msgs::JointState& msg);
	void jointStateCallback(const sensor_msgs::JointState& msg);

	void calculateFKClient(const sensor_msgs::JointState& joint_state, geometry_msgs::Pose& pose);
	void calculateIKClient(const geometry_msgs::Pose& pose, const std::vector<double>& seed_state, sensor_msgs::JointState& solution);
private:
	ros::NodeHandle nh;

	ros::Subscriber joint_state_sub;
	ros::Subscriber pose_cmd_sub;
	ros::Subscriber joint_cmd_sub;

	ros::ServiceClient calculate_fk_client;
	ros::ServiceClient calculate_ik_client;

	bool ready;

	double joint_upper_limit[ELFIN_DOF_JOINTS] = {3.14,  2.35,  2.61,  3.14,  2.56,  3.14};
	double joint_lower_limit[ELFIN_DOF_JOINTS] = {-3.14, -2.35, -2.61, -3.14, -2.56, -3.14};
	std::vector<double> joint_position = {0, 0, 0, 0, 0, 0};
	std::vector<double> joint_velocity = {0, 0, 0, 0, 0, 0};
	std::vector<std::string> joint_names = {
		"elfin_joint1",
		"elfin_joint2",
		"elfin_joint3",
		"elfin_joint4",
		"elfin_joint5",
		"elfin_joint6"
	};

	Client* robot_client;
	control_msgs::FollowJointTrajectoryGoal goal;

	sensor_msgs::JointState current_joint_state;
	geometry_msgs::Pose previous_pose_cmd;
	std::vector<double> previous_joints;

	// Interpolation Variables
	double exec_time;
	uint total_points;

	// Security variables
	bool secure;
	std::vector<double> joint_position_threshold = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
	std::vector<double> joint_position_difference = {0, 0, 0, 0, 0, 0};
	std::vector<double> joint_velocity_limit = {1.57, 1.57, 1.57, 1.57, 1.57, 1.57};
	std::vector<double> previous_joint_position = {0, 0, 0, 0, 0, 0};
	double velocity_scale = 0.25;
	std::vector<double> times;
};