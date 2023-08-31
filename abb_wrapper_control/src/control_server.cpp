/* CONTROL SERVER - Creates all necessary service servers to command Abb robot
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "abb_wrapper_control/ArmControl.h"
#include "abb_wrapper_control/PosePlan.h"
#include "abb_wrapper_control/SlerpPlan.h"
#include "abb_wrapper_control/JointPlan.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "abb_control_server");

    ros::NodeHandle nh_;
 
    // Get the params from the loaded YAML file

    std::string group_name;
	std::string end_effector_name;
    std::string pose_plan_service_name;
    std::string slerp_plan_service_name;
    std::string arm_control_service_name;
    std::string arm_wait_service_name;
    std::string joint_plan_service_name;
    
    //Move group name

	if(!nh_.getParam("/abb/group_name", group_name)){
        ROS_ERROR("Failed to load the move group name!");
    };

    ROS_INFO("The move group name is: %s", group_name.c_str());
 
    //End-effector name use for planning
    
	if(!nh_.getParam("/abb/end_effector_name", end_effector_name)){
        ROS_ERROR("Failed to load the end_effector_name!");
    };

    // Pose plan name service use for planning
    
	if(!nh_.getParam("/abb/pose_plan_service_name", pose_plan_service_name)){
        ROS_ERROR("Failed to load the pose_plan_service_name!");
    };

    // Slerp plan name service use for planning

    if(!nh_.getParam("/abb/slerp_plan_service_name", slerp_plan_service_name)){
        ROS_ERROR("Failed to load the slerp_plan_service_name!");
    };

    // Arm control service name use for controlling the robot

    if(!nh_.getParam("/abb/arm_control_service_name", arm_control_service_name)){
        ROS_ERROR("Failed to load the arm_control_service_name!");
    };

    if(!nh_.getParam("/abb/arm_wait_service_name", arm_wait_service_name)){
        ROS_ERROR("Failed to load the arm_wait_service_name!");
    };

    // Joint Plan service name
    
    if(!nh_.getParam("/abb/joint_plan_service_name", joint_plan_service_name)){
        ROS_ERROR("Failed to load the joint_plan_service_name!");
    };

	/*------------------ Arm Control --------------------*/

    ROS_INFO("Creating the arm client pointer");
    std::string arm_jt_topic = "/robot_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic, true));
    
    ROS_INFO("Creating the arm control object");
    ArmControl arm_control_obj(nh_, arm_client_ptr_);

    /*-------------------------------------------------------*/


    /*------------------- Pose Plan ------------------------*/

    ROS_INFO("Creating the pose plan object");
    PosePlan pose_plan_obj(nh_, group_name, end_effector_name);

    /*-------------------------------------------------------*/

    
    /*------------------- Slerp Plan -------------------------*/

    ROS_INFO("Creating the slerp plan object");
    SlerpPlan slerp_plan_obj(nh_, group_name, end_effector_name, 60);

    /*-------------------------------------------------------*/

    
    /*-------------------- Joint Plan ------------------------*/

    ROS_INFO("Creating the joint plan object");
    JointPlan joint_plan_obj(nh_, group_name);

    /*-------------------------------------------------------*/

    ROS_INFO("Advertising the services");

    //
    ros::ServiceServer pose_service = nh_.advertiseService(pose_plan_service_name, &PosePlan::call_pose_plan, &pose_plan_obj);
    ros::ServiceServer slerp_service = nh_.advertiseService(slerp_plan_service_name, &SlerpPlan::call_slerp_plan, &slerp_plan_obj);
    ros::ServiceServer arm_service = nh_.advertiseService(arm_control_service_name, &ArmControl::call_arm_control, &arm_control_obj);
    ros::ServiceServer arm_wait_service = nh_.advertiseService(arm_wait_service_name, &ArmControl::call_arm_wait, &arm_control_obj);
    ros::ServiceServer joint_service = nh_.advertiseService(joint_plan_service_name, &JointPlan::call_joint_plan, &joint_plan_obj);

    ROS_INFO("The main service server is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}