/* JOINT CONTROL - Uses moveit movegroupinterface to plan towards a joint configuration
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli 
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipit.it  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "abb_wrapper_control/JointPlan.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

JointPlan::JointPlan(ros::NodeHandle& nh_, std::string group_name_){
        
        ROS_INFO("Starting to create JointPlan object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->group_name = group_name_;

        ROS_INFO("Finished creating JointPlan object");
}

JointPlan::~JointPlan(){
    
    // Nothing to do here yet
}

// This is the callback function of the joint plan service
bool JointPlan::call_joint_plan(abb_wrapper_msgs::joint_plan::Request &req, abb_wrapper_msgs::joint_plan::Response &res){

    // Setting up things
    if(!this->initialize(req)){
        ROS_ERROR("Could not initialize JointPlan object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal joint
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in JointPlan object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return the computed trajectory
    res.computed_trajectory = this->computed_trajectory;
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool JointPlan::initialize(abb_wrapper_msgs::joint_plan::Request &req){

    // Converting the float array of request to std vector
    this->joint_goal = req.joint_goal;
    this->past_trajectory = req.past_trajectory;
    this->flag_state = req.flag_state;
    
    if(!this->flag_state && this->past_trajectory.points.empty()){
        ROS_ERROR("The past trajectory is empty! You CAN'T plan from the last point of the previous computed trajectory!");
        return false;
    }
    // Print the goal end-effector pose
    if(DEBUG){
        std::cout << "Requested joint configuration goal: [ ";
        for(auto i : this->joint_goal) std::cout << i << " ";
        std::cout << "]" << std::endl;
    }
    return true;
}

// Performs motion planning for the joints towards goal
bool JointPlan::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);

    //
    // group.setPlanningPipelineId("pilz_industrial_motion_planner");
    // group.setPlannerId("PTP");

    // Getting current joint state
    ros::spinOnce();                                    // May not be necessary
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = current_state->getJointModelGroup(this->group_name);
    
    /* If VISUAL is enabled */
    #ifdef VISUAL

    // Visual tools
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Loading the remote control for visual tools and promting a message
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    #endif

	// Printing the planning group frame and the group ee frame
	if(DEBUG) ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());

    // Setting the joint config target of the move group
    group.setJointValueTarget(this->joint_goal);

    // Planning to joint configuration
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 

    //if false planning from the last point of the computed trajectory
    if(!this->flag_state){
        int last_waypoint_index = this->past_trajectory.points.size() - 1;
        std::vector<double> last_waypoint_joint_values = this->past_trajectory.points[last_waypoint_index].positions;
        current_state->setJointGroupPositions(group.getName(), last_waypoint_joint_values);
        group.setStartState(*current_state);
        my_plan.start_state_.joint_state.position = last_waypoint_joint_values;
        this->flag_state = false;
        ROS_WARN("NOT Planning from the current state");
    }else {

        ROS_WARN("Planning from the current state");
    }
    
    //
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Motion Plan towards goal joint configuration %s.", success ? "SUCCEDED" : "FAILED");

    // If complete path is not achieved return false, true otherwise
	if(!success) return false;
    
    /* If VISUAL is enabled */
    // #ifdef VISUAL

    // ROS_INFO("Visualizing the computed plan as trajectory line.");
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    
    #ifdef PROMPT
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.");
    #endif

    // #endif

    // Saving the computed trajectory and returning true
    this->computed_trajectory = my_plan.trajectory_.joint_trajectory;
    return true;
}