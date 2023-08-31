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
    
    // At this point initializing completed
    return true;
}

// Service call function for arm control
bool AbbClient::call_arm_control_service(trajectory_msgs::JointTrajectory& computed_trajectory){
/**
    * @brief This functions allows to control the robot by sending the goal through the action interface.
    *
    * This function calls the arm control service server using the `arm_control_client` service client with the computed trajectory from the pose_plan, slerp_plan or joint_plan service.
    *
    * @param computed_trajectory [in] The computed trajectory ready to be executed.
    * 
    * @note Make sure the arm control server is properly initialized(all the ROS service server are initialized in the control_server.cpp)
    *       $: roslaunch abb_wrapper_control launchControlServer.launch
    *       
*/
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
/**
    * @brief This functions waits for the result from the action server for a maximum time defined in the wait_time.
    *
    * @param wait_time [in] Max time to block before returning.
    * 
    * @note Make sure the arm control server is properly initialized(all the ROS service server are initialized in the control_server.cpp)
    *       $: roslaunch abb_wrapper_control launchControlServer.launch
    *       
*/
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
/**
    * @brief This functions implements a trajectory planning from the current state of the robot or from the last point of the previous past trajectory toward a specified target Cartesian goal.
    *
    * This function calls the pose service server using the `pose_client` service client with the specified Cartesian goal,
    * boolean flag, past and computed trajectory. It populates the computed trajectory based on the service response.
    *
    * @param goal_pose [in] The Cartesian target goal you want to plan toward.
    * @param start_pose [in] The Cartesian initial pose you want to plan from.
    *                        (Setting zero pose as starting from present current state of the robot).
    * @param is_goal_relative [in] If the goal is relative is set to true, otherwise use the default values set to false.
    * @param past_trajectory [in] The past trajectory.
    * @param computed_trajectory [out] The computed trajectory received from the service server.
    * @return Returns the answer from the pose plan service server.
    *
    * @note Make sure the pose plan server is properly initialized(all the ROS service server are initialized in the control_server.cpp)
    *       $: roslaunch abb_wrapper_control launchControlServer.launch
    *       
*/
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
bool AbbClient::call_joint_service(std::vector<double> joint_goal, bool flag_state, trajectory_msgs::JointTrajectory& past_trajectory, trajectory_msgs::JointTrajectory& computed_trajectory){
/**
    * @brief This functions implements a trajectory planning from the current state of the robot or from the last point of the previous past trajectory toward a joint position goal.
    *
    * This function calls the joint service server using the `joint_client` service client with the specified joint goal,
    * boolean flag, past and computed trajectory. It populates the computed trajectory based on the service response.
    *
    * @param joint_goal [in] The joint position goal you want to plan toward.
    * @param flag_state [in] A boolean indicating if you want to plan from the current state or from the last point of the past trajectory: false, it doesn't plan from the current state; true, it plans from the current state.
    * @param past_trajectory [in] The past trajectory.
    * @param computed_trajectory [out] The computed joint trajectory received from the service server.
    * @return Returns the answer from the joint plan service server.
    *
    * @note Make sure the joint plan server is properly initialized(all the ROS service server are initialized in the control_server.cpp)
    *       $: roslaunch abb_wrapper_control launchControlServer.launch
    *       
*/

    // Creating and filling up the request
    abb_wrapper_msgs::joint_plan joint_plan_srv;
    joint_plan_srv.request.joint_goal = joint_goal;
    joint_plan_srv.request.flag_state = flag_state;
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
/**
    * @brief This functions implements a SLERP interpolation between the end-effector pose and goal pose to create homogeneous motion.
    *
    * This function calls the slerp service server using the `slerp_client` service client with the specified Cartesian goal,
    * boolean flag, past and computed trajectory. It populates the computed trajectory based on the service response.
    *
    * @param goal_pose [in] The cartesian target goal you want to plan toward.
    * @param start_pose [in] The cartesian initial pose you want to plan from.
    *                   (Setting zero pose as starting from present current state of the robot).
    * @param is_goal_relative [in] If the goal is relative is set to true, otherwise use the default values set to false.
    * @param past_trajectory [in] The past trajectory.
    * @param computed_trajectory [out] The computed trajectory received from the service server.
    * @return Returns the answer from the slerp plan service server.
    *
    * @note Make sure the pose plan server is properly initialized(all the ROS service server are initialized in the control_server.cpp)
    *       $: roslaunch abb_wrapper_control launchControlServer.launch
    *       
*/
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

bool AbbClient::call_closing_gripper(bool close){
   
   std_srvs::Trigger trigger_srv;
   
    // Calling the service
    if(close && !this->grip_in_client.call(trigger_srv)){
        ROS_ERROR("Failed to contact the grip_in server. Returning...");
        return false;
    }

    return trigger_srv.response.success;
}

bool AbbClient::call_opening_gripper(bool open){
   
   std_srvs::Trigger trigger_srv;

    // Calling the service
    if(open && !this->grip_out_client.call(trigger_srv)){
        ROS_ERROR("Failed to contact the grip_out server. Returning...");
        return false;
    }

    return trigger_srv.response.success;
}



