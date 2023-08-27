/* ABBCLIENT - Contains all necessary objects and functions to call the services to 
control Abb
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it  */

#include "abb_wrapper_control/Abb_client.h"
#include <std_msgs/Duration.h>

AbbClient::AbbClient(){

    // Nothing to do here
}

AbbClient::AbbClient(ros::NodeHandle& nh_){

    // Initializing object
    if(!this->initialize(nh_)){
        ROS_ERROR("The AbbClient was not initialized successfully. Some servers are missing...");
        ros::shutdown();
    } 
}

AbbClient::~AbbClient(){
    
    // Nothing to do here yet
}

// Initializing function
bool AbbClient::initialize(ros::NodeHandle& nh_){

    // Initializing node handle
    this->nh = nh_;

    // Parse from YAML the name of the services

    if(!nh_.getParam("/abb/arm_control_service_name", this->arm_control_service_name)){
        ROS_ERROR("Failed to load the arm_control_service_name!");
    };

    if(!nh_.getParam("/abb/arm_wait_service_name", this->arm_wait_service_name)){
        ROS_ERROR("Failed to load the arm_wait_service_name!");
    };

    if(!nh_.getParam("/abb/pose_plan_service_name", this->pose_service_name)){
        ROS_ERROR("Failed to load the pose_plan_service_name!");
    };

    if(!nh_.getParam("/abb/slerp_plan_service_name", this->slerp_service_name)){
        ROS_ERROR("Failed to load the pose_plan_service_name!");
    };

    if(!nh_.getParam("/abb/joint_plan_service_name", this->joint_service_name)){
        ROS_ERROR("Failed to load the joint_service_name!");
    };
    
    // Parse gripper clients

    if(!nh_.getParam("/abb/closing_gripper_service_name", this->gripper_service_grip_in)){
        ROS_ERROR("Failed to load the closing_gripper_service_name!");
    };

    if(!nh_.getParam("/abb/opening_gripper_service_name", this->gripper_service_grip_out)){
        ROS_ERROR("Failed to load the opening_gripper_service_name!");
    };

    if(!nh_.getParam("/abb/camera_pose_service_name", this->camera_pose_service_name)){
        ROS_ERROR("Failed to load the camera_pose_service_name!");
    };

    // Initializing service clients after waiting

    if(!ros::service::waitForService(this->arm_control_service_name, ros::Duration(1.0))) return false;
    this->arm_control_client = this->nh.serviceClient<abb_wrapper_msgs::arm_control>(this->arm_control_service_name);

    if(!ros::service::waitForService(this->arm_wait_service_name, ros::Duration(1.0))) return false;
    this->arm_wait_client = this->nh.serviceClient<abb_wrapper_msgs::arm_wait>(this->arm_wait_service_name);
   
    if(!ros::service::waitForService(this->pose_service_name, ros::Duration(1.0))) return false;
    this->pose_client = this->nh.serviceClient<abb_wrapper_msgs::pose_plan>(this->pose_service_name);
    
    if(!ros::service::waitForService(this->slerp_service_name, ros::Duration(1.0))) return false;
    this->slerp_client = this->nh.serviceClient<abb_wrapper_msgs::slerp_plan>(this->slerp_service_name);

    if(!ros::service::waitForService(this->joint_service_name, ros::Duration(1.0))) return false;
    this->joint_client = this->nh.serviceClient<abb_wrapper_msgs::joint_plan>(this->joint_service_name);
    
    if(!ros::service::waitForService(this->gripper_service_grip_in, ros::Duration(1.0))) return false;
    this->grip_in_client = this->nh.serviceClient<std_srvs::Trigger>(this->gripper_service_grip_in);
    
    if(!ros::service::waitForService(this->gripper_service_grip_out, ros::Duration(1.0))) return false;
    this->grip_out_client = this->nh.serviceClient<std_srvs::Trigger>(this->gripper_service_grip_out);
    
    if(!ros::service::waitForService(this->camera_pose_service_name, ros::Duration(1.0))) return false;
    this->camera_pose_client = this->nh.serviceClient<abb_wrapper_msgs::pose_camera>(this->camera_pose_service_name);
 
    // At this point initializing completed
    return true;
}

// Service call function for arm control
bool AbbClient::call_arm_control_service(trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    abb_wrapper_msgs::arm_control arm_control_srv;
    arm_control_srv.request.computed_trajectory = computed_trajectory;

    // Calling the service
    if(!this->arm_control_client.call(arm_control_srv)){
        ROS_ERROR("Failed to contact the arm control server. Returning...");
        return false;
    }

    return arm_control_srv.response.answer;
}

// Service call function for arm wait
bool AbbClient::call_arm_wait_service(ros::Duration wait_time){

    // Creating and filling up the request
    abb_wrapper_msgs::arm_wait arm_wait_srv;
    std_msgs::Duration wait_duration_msg;
    wait_duration_msg.data = wait_time;
    arm_wait_srv.request.wait_duration = wait_duration_msg;

    // Calling the service
    if(!this->arm_wait_client.call(arm_wait_srv)){
        ROS_ERROR("Failed to contact the arm wait server. Returning...");
        return false;
    }

    return arm_wait_srv.response.answer;
}

// Service call function for pose plan
bool AbbClient::call_pose_service(const geometry_msgs::Pose& goal_pose, const geometry_msgs::Pose& start_pose, bool is_goal_relative, 
                                            trajectory_msgs::JointTrajectory& computed_trajectory, const trajectory_msgs::JointTrajectory& past_trajectory){

    // Creating and filling up the request
    abb_wrapper_msgs::pose_plan pose_plan_srv;
    pose_plan_srv.request.goal_pose = goal_pose;
    pose_plan_srv.request.start_pose = start_pose;
    pose_plan_srv.request.is_goal_relative = is_goal_relative;
    pose_plan_srv.request.past_trajectory = past_trajectory;

    // Calling the service
    if(!this->pose_client.call(pose_plan_srv)){
        ROS_ERROR("Failed to contact the pose plan server. Returning...");
        return false;
    }

    computed_trajectory = pose_plan_srv.response.computed_trajectory;
    return pose_plan_srv.response.answer;
}

// Service call function for joint plan
bool AbbClient::call_joint_service(std::vector<double> joint_goal, bool is_true, trajectory_msgs::JointTrajectory& past_trajectory, trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    abb_wrapper_msgs::joint_plan joint_plan_srv;
    joint_plan_srv.request.joint_goal = joint_goal;
    joint_plan_srv.request.is_true = is_true;
    joint_plan_srv.request.past_trajectory = past_trajectory;

    // Calling the service
    if(!this->joint_client.call(joint_plan_srv)){
        ROS_ERROR("Failed to contact the joint plan server. Returning...");
        return false;
    }
    
    computed_trajectory = joint_plan_srv.response.computed_trajectory;

    return joint_plan_srv.response.answer;
}

// Service call function for slerp plan
bool AbbClient::call_slerp_service(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                            trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory){

    // Creating and filling up the request
    abb_wrapper_msgs::slerp_plan slerp_plan_srv;
    slerp_plan_srv.request.goal_pose = goal_pose;
    slerp_plan_srv.request.start_pose = start_pose;
    slerp_plan_srv.request.is_goal_relative = is_goal_relative;
    slerp_plan_srv.request.past_trajectory = past_trajectory;

    // Calling the service
    if(!this->slerp_client.call(slerp_plan_srv)){
        ROS_ERROR("Failed to contact the slerp plan server. Returning...");
        return false;
    }

    computed_trajectory = slerp_plan_srv.response.computed_trajectory;
    return slerp_plan_srv.response.answer;
}

bool AbbClient::call_closing_gripper(std_msgs::Bool& close){
   
   std_srvs::Trigger trigger_srv;
   
    // Calling the service
    if(close.data && !this->grip_in_client.call(trigger_srv)){
        ROS_ERROR("Failed to contact the grip_in server. Returning...");
        return false;
    }

    return trigger_srv.response.success;
}

bool AbbClient::call_opening_gripper(std_msgs::Bool& open){
   
   std_srvs::Trigger trigger_srv;

    // Calling the service
    if(open.data && !this->grip_out_client.call(trigger_srv)){
        ROS_ERROR("Failed to contact the grip_out server. Returning...");
        return false;
    }

    return trigger_srv.response.success;
}
bool AbbClient::call_camera_pose(std_msgs::Bool& is_request_arrived, geometry_msgs::Pose& object_pose){

    abb_wrapper_msgs::pose_camera pose_camera_srv;
    pose_camera_srv.request.is_request_arrived = is_request_arrived.data;

    // Calling the service
    if(!this->camera_pose_client.call(pose_camera_srv)){
        ROS_ERROR("Failed to contact the camera_pose server. Returning...");
        return false;
    }
    
    
    object_pose = pose_camera_srv.response.object_grasp_pose;
    return pose_camera_srv.response.answer;
}


