/* SLERP CONTROL - Uses SLERP interpolation between ee pose and goal pose to create homogeneous motion
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "abb_wrapper_control/SlerpPlan.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

SlerpPlan::SlerpPlan(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_,  int n_wp_){
        
        ROS_INFO("Starting to create SlerpPlan object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->end_effector_name = end_effector_name_;
        this->group_name = group_name_;

        // Setting number of waypoints 
        this->n_wp = n_wp_;

        ROS_INFO("Finished creating SlerpPlan object");
}

SlerpPlan::~SlerpPlan(){
    
    // Nothing to do here yet
}

// This is the callback function of the slerp plan service
bool SlerpPlan::call_slerp_plan(abb_wrapper_msgs::slerp_plan::Request &req, abb_wrapper_msgs::slerp_plan::Response &res){

    // Setting up things
    if(!this->initialize(req.goal_pose, req.start_pose, req.is_goal_relative, req.past_trajectory)){
        ROS_ERROR("Could not initialize SlerpPlan object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal pose
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in SlerpPlan object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return the computed trajectory
    res.computed_trajectory = this->computed_trajectory;
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool SlerpPlan::initialize(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, trajectory_msgs::JointTrajectory past_trajectory){

    // Getting the current ee transform
    try {
		this->tf_listener.waitForTransform("/single_yumi_base_link", this->end_effector_name, ros::Time(0), ros::Duration(10.0) );
		this->tf_listener.lookupTransform("/single_yumi_base_link", this->end_effector_name, ros::Time(0), this->stamp_ee_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s", ex.what());
      	ros::Duration(1.0).sleep();
        return false;
    }

    tf::Transform ee_transform(this->stamp_ee_transform.getRotation(), this->stamp_ee_transform.getOrigin());
    tf::transformTFToEigen(ee_transform, this->end_effector_state);

	// Print the current end-effector pose
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Translation: " << this->end_effector_state.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Rotation: " << this->end_effector_state.rotation());

	// Setting the start pose
    if (this->is_really_null_pose(start_pose)){
        ROS_WARN("The start pose is NULL! PLANNING FROM CURRENT POSE!");
        this->startAff = this->end_effector_state;
        this->was_really_null = true;
    } else {
        tf::poseMsgToEigen(start_pose, this->startAff);
        this->was_really_null = false;
    }
	
	// Setting the goal pose
    tf::poseMsgToEigen(goal_pose, this->goalAff);

	// If the goal is relative, get the global goal and start poses by multiplying it with ee pose (end_effector_state)
	if(is_goal_relative){
        this->goalAff = this->end_effector_state * this->goalAff;
		this->startAff = this->end_effector_state * this->startAff;
	}

    // Setting the past trajectory
    this->past_trajectory = past_trajectory;

    // Print the goal end-effector pose
    if(DEBUG) ROS_INFO_STREAM("Endeffector goal Translation: " << this->goalAff.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector goal Rotation: " << this->goalAff.linear());

    return true;
}

// Performs motion planning for the end-effector towards goal
bool SlerpPlan::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ros::spinOnce();        // May not be necessary

    // Getting the robot joint model
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(this->group_name);

    // Visual tools
    moveit_visual_tools::MoveItVisualTools visual_tools("single_yumi_base_link");
    visual_tools.deleteAllMarkers();

    // Loading the remote control for visual tools and promting a message
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    #endif

    // Set the desired planning frame

    std::string new_planning_frame = "single_yumi_base_link";
    group.setPoseReferenceFrame(new_planning_frame);

	// Printing the planning group frame and the group ee frame
    if(DEBUG) ROS_INFO("Pose Reference Frame: %s", group.getPoseReferenceFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());
   
	// Calling the waypoint creator with start and goal poses
	std::vector<geometry_msgs::Pose> cart_waypoints;
	this->computeWaypointsFromPoses(this->startAff, this->goalAff, cart_waypoints);

    // Setting the start state in the moveit group
    robot_state::RobotState start_state(*group.getCurrentState());
    if (!this->was_really_null) {
        std::vector<double> last_joints = this->past_trajectory.points.back().positions;
        // geometry_msgs::Pose starting_pose;
        // tf::poseEigenToMsg(this->startAff, starting_pose);
        start_state.setJointGroupPositions(joint_model_group, last_joints);
        group.setStartState(start_state);
    }

    // Scale the velocity and acceleration of the computed trajectory
    const double velocity_scaling_factor = 0.1; // Set your desired velocity scaling factor
    const double acceleration_scaling_factor = 0.1; // Set your desired acceleration scaling factor

	// Planning for the waypoints path
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(cart_waypoints, 0.01, 0.0, trajectory);
    
    //
    robot_trajectory::RobotTrajectory rt(start_state.getRobotModel(), this->group_name);
    
    rt.setRobotTrajectoryMsg(start_state, trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    
    bool success = totg.computeTimeStamps(rt, velocity_scaling_factor, acceleration_scaling_factor);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    rt.getRobotTrajectoryMsg(trajectory);
    //
    this->computed_trajectory = trajectory.joint_trajectory;

	ROS_INFO("Plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // If complete path is not achieved return false, true otherwise
	if(fraction != 1.0) return false;

    std::cout << "FOUND COMPLETE PLAN FOR WAYPOINTS!!!" << std::endl;

    /* If VISUAL is enabled */
    // #ifdef VISUAL

    // ROS_INFO("Visualizing the computed plan as trajectory line.");
    // visual_tools.publishAxisLabeled(cart_waypoints.back(), "goal pose");
    // visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    // visual_tools.trigger();
    
    #ifdef PROMPT
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.");
    #endif

    // #endif

    return true;
}

// Computes waypoints using SLERP from two poses
void SlerpPlan::computeWaypointsFromPoses(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& goal_pose, std::vector<geometry_msgs::Pose>& waypoints){
    
    // Compute waypoints as linear interpolation (SLERP for rotations) between the two poses
	Eigen::Affine3d wp_eigen;
	geometry_msgs::Pose current_wp;

	Eigen::Vector3d start_vec = start_pose.translation();
	Eigen::Vector3d goal_vec = goal_pose.translation();
	Eigen::Vector3d diff_vec = goal_vec - start_vec;

	Eigen::Quaterniond start_quat(start_pose.linear());
	Eigen::Quaterniond goal_quat(goal_pose.linear());
	Eigen::Quaterniond diff_quat;

    // Setting the number of wp according to diff_vec
    this->real_n_wp = std::floor(diff_vec.norm() * this->n_wp);
    if(DEBUG) ROS_INFO_STREAM("The norm of the diff_vec is " << diff_vec.norm() << 
        ", so the new number of waypoints is " << this->real_n_wp << ".");

	for(int j = 1; j <= this->real_n_wp; j++){
		wp_eigen.translation() = start_vec + (diff_vec / this->real_n_wp) * j;
		diff_quat = start_quat.slerp(double(j)/double(this->real_n_wp), goal_quat);
		wp_eigen.linear() = diff_quat.toRotationMatrix();
		tf::poseEigenToMsg (wp_eigen, current_wp);
		waypoints.push_back(current_wp);

        if(DEBUG){
            std::cout << "WAYPOINT NUMBER " << j << "." << std::endl;
		    std::cout << "WP R: \n" << wp_eigen.linear() << std::endl;
		    std::cout << "WP t: \n" << wp_eigen.translation() << std::endl;
        }
    }
}