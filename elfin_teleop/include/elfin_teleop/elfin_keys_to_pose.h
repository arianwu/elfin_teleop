#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

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

const std::string KEYS_TOPIC = "keys";
const std::string POSE_CMD_TOPIC = "pose_cmd";
const std::string JOINT_STATE_TOPIC = "joint_states";

class ElfinKeysToPoseNode {
public:
	ElfinKeysToPoseNode();
	~ElfinKeysToPoseNode();

	void keysCallback(const std_msgs::String::ConstPtr& msg);
	void jointStateCallback(const sensor_msgs::JointState& msg);

	void denavitHartenberg(double th, double d, double alpha, double a, Eigen::Matrix4d &dh);
	void forwardKinematics(std::vector<double>& q, geometry_msgs::Point &trans, geometry_msgs::Quaternion &orient);

private:
	ros::NodeHandle nh;

	ros::Subscriber keys_sub;
	ros::Subscriber joint_state_sub;
	ros::Publisher pose_cmd_pub;

	bool ready;

	sensor_msgs::JointState current_joint_state;

	geometry_msgs::Pose desired_pose;
	geometry_msgs::Point desired_position;
	geometry_msgs::Quaternion desired_orientation;
};