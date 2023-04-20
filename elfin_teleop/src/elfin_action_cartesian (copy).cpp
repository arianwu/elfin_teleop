#include "elfin_teleop/elfin_action_cartesian.h"

// TODO
// RESOLVE SEGMENTATION FAULT IN JOINT VELOCITY LIMITATION

ElfinActionCartesianNode::ElfinActionCartesianNode(){
	// Initialize Variable
	exec_time = 1;
	ready = false;
	times.resize(6);
	
	// ROS Infrastructures
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &ElfinActionCartesianNode::jointStateCallback, this);
	pose_cmd_sub = nh.subscribe(POSE_CMD_TOPIC, 1, &ElfinActionCartesianNode::poseCmdCallback, this);

	// Wait for things to happen
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Service things
	ROS_INFO("Waiting for calculate_ik and calculate_fk...");
	ros::service::waitForService(CALCULATE_FK_SERVER_TOPIC);
	ros::service::waitForService(CALCULATE_IK_SERVER_TOPIC);
	ROS_INFO("Connected to calculate_ik and calculate_fk");

	calculate_fk_client = nh.serviceClient<elfin_ik_solver_msgs::CalculateFK>(CALCULATE_FK_SERVER_TOPIC);
	calculate_ik_client = nh.serviceClient<elfin_ik_solver_msgs::CalculateIK>(CALCULATE_IK_SERVER_TOPIC);

	// Prepare interpolation variables
	calculateFKClient(current_joint_state, previous_pose_cmd);
	previous_joints = current_joint_state.position;

	// Action Library for ros_control
	robot_client = new Client("elfin_arm_controller/follow_joint_trajectory", true);

	ROS_INFO("Waiting for server...");
	robot_client->waitForServer();
	ROS_INFO("Connected to server");

	goal.trajectory.joint_names = joint_names;
	goal.trajectory.points.resize(1);
	goal.trajectory.points[0].positions.resize(6);
	goal.trajectory.points[0].velocities.resize(6);
	goal.trajectory.points[0].velocities = joint_velocity;
}

ElfinActionCartesianNode::~ElfinActionCartesianNode(){
	delete robot_client;
	nh.shutdown();
}

void ElfinActionCartesianNode::poseCmdCallback(const geometry_msgs::Pose& msg){
	if (!ready)
		return;

	// Initialize actual position and orientation
	tf2::Vector3 actual_position;
	tf2::Quaternion actual_orientation;
	geometry_msgs::Pose actual_pose;

	// Obtain current position and orientation (quat)      has to be tf
	tf2::Vector3 desired_position(msg.position.x, msg.position.y, msg.position.z);
	tf2::Quaternion desired_orientation(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

	// Obtain previous position and orientation (quat)     has to be tf
	tf2::Vector3 previous_position(previous_pose_cmd.position.x, previous_pose_cmd.position.y, previous_pose_cmd.position.z);
	tf2::Quaternion previous_orientation(previous_pose_cmd.orientation.x, previous_pose_cmd.orientation.y, previous_pose_cmd.orientation.z, previous_pose_cmd.orientation.w);	

	// Define # of points
	uint total_points = 5;
	// Total time
	double total_time = exec_time;
	// Resize points
	goal.trajectory.points.resize(total_points);
	double actual_time = 0;
	double time_step = 1 / (float)(total_points - 1);
	// for loop
	for (size_t i=0; i<total_points; i++){
		double s = i / (float)(total_points - 1);
		actual_position = previous_position.lerp(desired_position, s); 
		actual_orientation = previous_orientation.slerp(desired_orientation, s);
		actual_pose.position.x = actual_position.getX();
		actual_pose.position.y = actual_position.getY();
		actual_pose.position.z = actual_position.getZ();
		actual_pose.orientation.x = actual_orientation.getX();
		actual_pose.orientation.y = actual_orientation.getY();
		actual_pose.orientation.z = actual_orientation.getZ();
		actual_pose.orientation.w = actual_orientation.getW();
		sensor_msgs::JointState solution;
		calculateIKClient(actual_pose, previous_joints, solution);

		actual_time += time_step;

		trajectory_msgs::JointTrajectoryPoint point;
		if (solution.position.size() > 0){
			// SECURITY STUFF TO LIMIT VELOCITY IF GOAL IS TOO FAR AWAY (avoids big movements at great speed)
			{
				bool limit = false;
				for(size_t i=0; i<ELFIN_DOF_JOINTS; i++)
					if (abs(solution.position[i] - previous_joints[i]) > joint_position_threshold[i])
						limit = true;
				if (limit){
					// Calculate the times required to reach goal using max velocity
					for (size_t i=0; i<ELFIN_DOF_JOINTS; i++)
						times[i] = abs(solution.position[i] - previous_joints[i]) / (joint_velocity_limit[i] * velocity_scale);
					// If max time required is greater than actual execution time, change execution time
					double max_time = *max_element(times.begin(), times.end());
					if (max_time > time_step){
						actual_time += max_time - time_step;
						ROS_WARN("EXECUTION TIME FOR POINT %ld ADJUSTED FROM %.2f TO %.2f", i+1, time_step, max_time); 
					}
				}
			} // END SECURITY STUFF
			point.positions = solution.position;
			previous_joints = solution.position;
		}
		else{
			point.positions = previous_joints;
		}

		point.time_from_start = ros::Duration(actual_time);
		goal.trajectory.points[i] = point;
	}
	
	// update previous pose cmd
	previous_pose_cmd = msg;

	// Send goal
	//goal.trajectory.header.stamp = ros::Time::now(); // causes this warning Dropping first 1 trajectory point(s) out of 5, as they occur before the current time.
	robot_client->sendGoal(goal);
	robot_client->waitForResult();
	ROS_INFO("TRAJECTORY DONE");
	
}

void ElfinActionCartesianNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void ElfinActionCartesianNode::calculateFKClient(const sensor_msgs::JointState& joint_state, geometry_msgs::Pose& pose){
	elfin_ik_solver_msgs::CalculateFK srv;
	srv.request.joint_state = joint_state;
	if (!calculate_fk_client.call(srv))
		ROS_ERROR("Failed to call service calculate_fk");
	pose = srv.response.pose;
}

void ElfinActionCartesianNode::calculateIKClient(const geometry_msgs::Pose& pose, const std::vector<double>& seed_state, sensor_msgs::JointState& solution){
	elfin_ik_solver_msgs::CalculateIK srv;
	srv.request.desired_pose = pose;
	srv.request.seed_state = seed_state;
	if (!calculate_ik_client.call(srv))
		ROS_ERROR("Failed to call service calculate_ik");
	if (srv.response.found.data)
		solution = srv.response.solution;
	else
		solution = sensor_msgs::JointState();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "elfin_action");

	ElfinActionCartesianNode node;
	ros::spin();

	return 0;
}