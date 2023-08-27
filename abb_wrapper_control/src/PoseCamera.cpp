/* POSE CAMERA - Get grasp pose for the dice w.r.t to single_yumi_base_link
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "abb_wrapper_control/PoseCamera.h"

PoseCamera::PoseCamera(ros::NodeHandle& nh_){
    
        ROS_INFO("Starting to create PoseCamera object");

        // Initializing node handle
        this->nh = nh_;

        ROS_INFO("Finished creating PoseCamera object");
}

PoseCamera::~PoseCamera(){
    
    // Nothing to do here yet
}

// This is the callback function of the pose plan service
bool PoseCamera::call_pose_camera(abb_wrapper_msgs::pose_camera::Request &req, abb_wrapper_msgs::pose_camera::Response &res){

    this->object_sub = this->nh.subscribe(this->object_topic_name, 1, &PoseCamera::get_object_pose, this);

    // Wait for a valid object pose message or timeout after 3 seconds
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>(this->object_topic_name, ros::Duration(3.0));
    
    ROS_INFO("The pose is respect to a valid frame!");
    // Fill the response
    ROS_INFO("The request is correct!");
    res.object_grasp_pose = this->object_pose_T.pose;
    res.answer = true;
    return res.answer;
}

void PoseCamera::get_object_pose(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    // Saving the message
    this->object_pose_T.pose = (*msg).pose;
}

bool PoseCamera::isPoseEmpty(const geometry_msgs::PoseStamped& object_pose) {
    return (object_pose.pose.position.x == 0.0 && object_pose.pose.position.y == 0.0 && object_pose.pose.position.z == 0.0) &&
           (object_pose.pose.orientation.x == 0.0 && object_pose.pose.orientation.y == 0.0 &&
            object_pose.pose.orientation.z == 0.0 && object_pose.pose.orientation.w == 0.0);
}


