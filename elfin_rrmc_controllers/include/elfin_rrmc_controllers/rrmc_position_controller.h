#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>

#include <urdf/model.h>

#include "elfin_kinematic_solver/elfin_kinematic_solver.h"
#include <eigen3/Eigen/Eigen>

#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>

namespace elfin_rrmc_controllers
{

class RRMCPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
	~RRMCPositionController();

	bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
	void starting(const ros::Time& time);
	void commandCB(const geometry_msgs::PoseConstPtr& msg);
	void update(const ros::Time& time, const ros::Duration& period);
	void stopping(const ros::Time& time);

	void forwardKinematics(const std::vector<double> joint_angles, Eigen::Vector3d& trans, Eigen::Quaterniond& quat);
	void analyticalJacobian(const std::vector<double> joint_angles, Eigen::MatrixXd& jacobian);
	void errorCalculation(const Eigen::Vector3d trans_actual, const Eigen::Quaterniond quat_actual, const Eigen::Vector3d trans_desired, const Eigen::Quaterniond quat_desired, Eigen::VectorXd& error);
	void dampedPseudoInverse(const Eigen::MatrixXd M, Eigen::MatrixXd& M_pinv);

private:
	// joint handles
	unsigned int n_joints_;
	std::vector<std::string> joint_names_;
	std::vector<hardware_interface::JointHandle> joints_;
	std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

	// kdl
	KDL::Tree kdl_tree_;
	KDL::Chain kdl_chain_;
	KDL::ChainJntToJacSolver* jnt_to_jac_solver;
	KDL::JntArray jnt_to_jac_solver_q;
    KDL::Jacobian jnt_to_jac_solver_J;

// FROM HERE DOWN NEED REVISION INITIALIZATION-WISE

	// ik fast
	elfin5_ikfast_simple_api::Elfin5KinematicSolver* elfin_solver;
	std::vector<double> joint_angles;
	Eigen::Vector3d elfin_solver_trans;
	Eigen::Matrix3d elfin_solver_orient;

	Eigen::Vector3d trans_actual, trans_desired;
	Eigen::Quaterniond quat_actual, quat_desired;
	Eigen::VectorXd error;
	Eigen::MatrixXd jacobian, jacobian_pinv;
	double damping, motion_rate;

	Eigen::VectorXd qdot_cmd, q_cmd, q, qdot;

	// Commands
	realtime_tools::RealtimeBuffer<geometry_msgs::Pose> commands_buffer_;

	// topic
	ros::Subscriber command_sub_;
	//boost::scoped_ptr<realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>> controller_state_pub_;
};

} // namespace rrmc_controllers