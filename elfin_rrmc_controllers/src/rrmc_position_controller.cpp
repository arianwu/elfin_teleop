#include "elfin_rrmc_controllers/rrmc_position_controller.h"

namespace elfin_rrmc_controllers
{

RRMCPositionController::~RRMCPositionController() {command_sub_.shutdown();}

bool RRMCPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n){
	// List of controlled joints
	if (!n.getParam("joints", joint_names_)){
		ROS_ERROR("Could not find joint name");
		return false;
	}
	n_joints_ = joint_names_.size();

	if (n_joints_ == 0) {
		ROS_ERROR("List of joint names is empty.");
		return false;
	}

	// urdf
	urdf::Model urdf;
	if (!urdf.initParam("robot_description")) {
		ROS_ERROR("Failed to parse urdf file");
		return false;
	}

	// joint handle
	for (int i=0; i<n_joints_; i++) {
		try {
			joints_.push_back(hw->getHandle(joint_names_[i]));
		}
		catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
			return false;
		}

		urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
		if (!joint_urdf) {
			ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
			return false;
		}
		joint_urdfs_.push_back(joint_urdf); 
	}

	// kdl parser
	if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)) {
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	// kdl chain
	std::string root_name, tip_name;
	if (!n.getParam("root_link", root_name)) {
		ROS_ERROR("Could not find root link name");
		return false;
	}
	if (!n.getParam("tip_link", tip_name)) {
		ROS_ERROR("Could not find tip link name");
		return false;
	}
	if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_)) {
		ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
		ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		ROS_ERROR_STREAM("  The segments are:");

		KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    	KDL::SegmentMap::iterator it;

    	for( it=segment_map.begin(); it != segment_map.end(); it++ )
      		ROS_ERROR_STREAM( "    "<<(*it).first);

    	return false;
	}

	// Obtain damping value
	if (!n.getParam("damping", damping)) {
		ROS_ERROR("Could not find damping value");
		return false;
	}

	// Obtain motion_rate value
	if (!n.getParam("motion_rate", motion_rate)) {
		ROS_ERROR("Could not find motion_rate value");
		return false;
	}

	// Initialize Elfin IK Fast Kinematic solver
	elfin_solver = new elfin5_ikfast_simple_api::Elfin5KinematicSolver();
	joint_angles.resize(n_joints_);

	// Initialize KDL Jacobian Solver
	jnt_to_jac_solver = new KDL::ChainJntToJacSolver(kdl_chain_);
	jnt_to_jac_solver_q.resize(n_joints_);
	jnt_to_jac_solver_J.resize(n_joints_);

	qdot_cmd.resize(n_joints_);
	q_cmd.resize(n_joints_);
	qdot.resize(n_joints_);
	q.resize(n_joints_);

	// Command buffer (?)
	commands_buffer_.writeFromNonRT(geometry_msgs::Pose());

	// Command subscriber (?)
	command_sub_ = n.subscribe<geometry_msgs::Pose>("command", 1, &RRMCPositionController::commandCB, this);
	
	return true;
}

void RRMCPositionController::starting(const ros::Time& time){
	// // testing area
	// Eigen::Vector3d trans;
	// Eigen::Quaterniond quat;
	// forwardKinematics({0, 1.57, 0, 0, 0, 0}, trans, quat);
	// std::cout<< trans <<std::endl;
	// std::cout<< quat.w() <<" "<< quat.x() <<" "<< quat.y() <<" "<< quat.z() <<std::endl;

	// Read position
	for (size_t i=0; i<n_joints_; i++){
		joint_angles[i] = joints_[i].getPosition();
		q_cmd(i) = joint_angles[i];
		q(i) = joint_angles[i];
	}

	// Forward Kinematics
	forwardKinematics(joint_angles, trans_actual, quat_actual);

	// Set initial command
	geometry_msgs::Pose initial_pose;
	initial_pose.position.x = trans_actual(0);
	initial_pose.position.y = trans_actual(1);
	initial_pose.position.z = trans_actual(2);
	initial_pose.orientation.w = quat_actual.w();
	initial_pose.orientation.x = quat_actual.x();
	initial_pose.orientation.y = quat_actual.y();
	initial_pose.orientation.z = quat_actual.z();

	commands_buffer_.writeFromNonRT(initial_pose);
}

void RRMCPositionController::commandCB(const geometry_msgs::PoseConstPtr& msg){
	commands_buffer_.writeFromNonRT(*msg);
}

void RRMCPositionController::update(const ros::Time& time, const ros::Duration& period){
	// Get Joint States

	// Get Actual Translation and Orientation

	// Get Command

	// Calculate Error

	// Calculate reference d_error

	// Calculate Jacobian

	// Calculate Pseudo-Inverse

	// Calculate velocities

	// Integrate

	// Send Commands


	// ********* OLD *********

	// Extract Command
	geometry_msgs::Pose pose_command = *commands_buffer_.readFromRT();
	trans_desired<< pose_command.position.x, pose_command.position.y, pose_command.position.z;
	quat_desired = Eigen::Quaterniond(pose_command.orientation.w, pose_command.orientation.x, pose_command.orientation.y, pose_command.orientation.z);

	// Get Joint States
	for (size_t i=0; i<n_joints_; i++)
		joint_angles[i] = joints_[i].getPosition();

	// Calculate Forward Kinematics ()
	forwardKinematics(joint_angles, trans_actual, quat_actual);

	// Calculate Error
	errorCalculation(trans_actual, quat_actual, trans_desired, quat_desired, error);

	// Calculate Jacobian
	analyticalJacobian(joint_angles, jacobian);

	// Calculate Pseudo-Inverse
	dampedPseudoInverse(jacobian, jacobian_pinv);

	// Calculate Velocities
	qdot_cmd = -motion_rate * jacobian_pinv * error;

	// Integrate Velocity (Only for position Interface)
	double dt = period.toSec();

	for (size_t i=0; i<n_joints_; i++)
		q(i) = joint_angles[i];

	q_cmd = q + dt * qdot_cmd;

	// Write Commands
	for (size_t i=0; i<n_joints_; i++){
		joints_[i].setCommand(q_cmd(i));
	}

	//printf("%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f \n\n", error(0), error(1), error(2), error(3), error(4), error(5), error(6));
	//printf("%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f \n\n", qdot_cmd(0), qdot_cmd(1), qdot_cmd(2), qdot_cmd(3), qdot_cmd(4), qdot_cmd(5));
	//std::cout<< dt <<std::endl<<std::endl;

	// Publish Controller State
}

void RRMCPositionController::stopping(const ros::Time& time){ }

void RRMCPositionController::forwardKinematics(const std::vector<double> joint_angles, Eigen::Vector3d& trans, Eigen::Quaterniond& quat){
	Eigen::Matrix3d orient;
	elfin_solver->getPositionFK(joint_angles, trans, orient);
	quat = Eigen::Quaterniond(orient);
}

void RRMCPositionController::analyticalJacobian(std::vector<double> joint_angles, Eigen::MatrixXd& jacobian){
	// Calculate Geometric Jacobian
	for (size_t i=0; i<n_joints_; i++)
		jnt_to_jac_solver_q(i) = joint_angles[i];

	jnt_to_jac_solver->JntToJac(jnt_to_jac_solver_q, jnt_to_jac_solver_J);

	//Calculate Forward Kinematics
	elfin_solver->getPositionFK(joint_angles, elfin_solver_trans, elfin_solver_orient);
	Eigen::Quaterniond q(elfin_solver_orient);

	// Calculate Transformation Matrix E
	Eigen::MatrixXd E;
	E.resize(7, 6);
    E<<1, 0, 0,        0,        0,        0,
       0, 1, 0,        0,        0,        0,
       0, 0, 1,        0,        0,        0,
       0, 0, 0, -q.x()/2, -q.y()/2, -q.z()/2,
       0, 0, 0,  q.w()/2,  q.z()/2, -q.y()/2,
       0, 0, 0, -q.z()/2,  q.w()/2,  q.x()/2,
       0, 0, 0,  q.y()/2, -q.x()/2,  q.w()/2;

    // Transform from geometric to analytical
    jacobian = E*jnt_to_jac_solver_J.data;
}

void RRMCPositionController::errorCalculation(const Eigen::Vector3d trans_actual, const Eigen::Quaterniond quat_actual, const Eigen::Vector3d trans_desired, const Eigen::Quaterniond quat_desired, Eigen::VectorXd& error){
	Eigen::Vector3d trans_error = trans_actual - trans_desired;
	Eigen::Quaterniond quat_error = quat_actual*quat_desired.inverse();
	error.resize(7);
	error<< trans_error, quat_error.w()-1, quat_error.x(), quat_error.y(), quat_error.z();
}

void RRMCPositionController::dampedPseudoInverse(const Eigen::MatrixXd M, Eigen::MatrixXd& M_pinv){
	M_pinv = M.transpose() * (M * M.transpose() + damping*damping*Eigen::MatrixXd::Identity(7, 7)).inverse() ;
}

} // namespace rrmc_controllers

PLUGINLIB_EXPORT_CLASS(elfin_rrmc_controllers::RRMCPositionController, controller_interface::ControllerBase)