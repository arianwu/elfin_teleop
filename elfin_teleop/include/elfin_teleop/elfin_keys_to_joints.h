#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <std_msgs/String.h>
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

const std::string KEYS_TOPIC = "keys";
const std::string JOINT_CMD_TOPIC = "joint_cmd";
const std::string JOINT_STATE_TOPIC = "joint_states";

class ElfinKeysToJointNode {
public:
	ElfinKeysToJointNode();
	~ElfinKeysToJointNode();

	void keysCallback(const std_msgs::String::ConstPtr& msg);
	void jointStateCallback(const sensor_msgs::JointState& msg);
	void limitJoints();

private:
	ros::NodeHandle nh;

	ros::Subscriber keys_sub;
	ros::Publisher joint_cmd_pub;
	ros::Subscriber joint_state_sub;

	sensor_msgs::JointState joint_state_cmd;
	sensor_msgs::JointState current_joint_state;

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
};