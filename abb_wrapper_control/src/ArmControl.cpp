/* ARM CONTROL - Uses actionlib to control the arm
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "abb_wrapper_control/ArmControl.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

ArmControl::ArmControl(ros::NodeHandle& nh_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_){
        
        ROS_INFO("Starting to create ArmControl object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing the arm client
        this->arm_client_ptr = arm_client_ptr_;

        ROS_INFO("Finished creating ArmControl object");
}

ArmControl::~ArmControl(){
    
    // Nothing to do here yet
}

// This is the callback function of the arm control service
bool ArmControl::call_arm_control(abb_wrapper_msgs::arm_control::Request &req, abb_wrapper_msgs::arm_control::Response &res){

    // Saving the callback msg (Here we hope that the traj has been created correctly)
    this->computed_trajectory = req.computed_trajectory;

    // Sending the trajectory to hand
    if(!this->sendJointTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from HandControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return true
    res.answer = true;
    return true;
}

// Sends trajectory to the joint_traj controller
bool ArmControl::sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory){
    
    // Waiting for the arm server to be ready
    if(!this->arm_client_ptr->waitForServer(ros::Duration(1.0))){
        ROS_ERROR("The arm client is taking too much to get ready. Returning...");
        return false;
    }

    // Setting the most recent time to the trajectory header
    std_msgs::Header empty_header;
    trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
    ROS_INFO_STREAM("In ArmControl::sendJointTrajectory, the traj header stamp is " << trajectory.header.stamp
        << " and the time_from_start of first point is " << trajectory.points[1].time_from_start << ".");

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->arm_client_ptr->sendGoal(goalmsg);

    // Not waiting for result here as it would be blocking

    return true;
}


// This is the callback function of the arm wait service
bool ArmControl::call_arm_wait(abb_wrapper_msgs::arm_wait::Request &req, abb_wrapper_msgs::arm_wait::Response &res){
     
     if(!this->arm_client_ptr->waitForResult(req.wait_duration.data)){
        ROS_ERROR("The arm client is taking too to complete goal execution. Returning...");
        res.answer = false;
        return false;
    }else{
        ROS_WARN("Everything seems ok!");
    }

    res.answer = true;
    return true;
}
