/* ABB CLIENT - Contains all necessary objects and functions to call the services to 
control Abb
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

// Custom msg and srv includes
#include "abb_wrapper_msgs/joint_plan.h"
#include "abb_wrapper_msgs/pose_plan.h"
#include "abb_wrapper_msgs/slerp_plan.h"
#include "abb_wrapper_msgs/arm_wait.h"
#include "abb_wrapper_msgs/arm_control.h"

// srv include for opening and closing the gripper
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class AbbClient {

    /// public variables and functions ------------------------------------------------------------
	public:
        AbbClient();

		AbbClient(ros::NodeHandle& nh_);

        ~AbbClient();

        // Initializing function
        bool initialize(ros::NodeHandle& nh_);

        // Service call function for arm control
        bool call_arm_control_service(trajectory_msgs::JointTrajectory& computed_trajectory);

        // Service call function for arm wait
        bool call_arm_wait_service(ros::Duration wait_time);

        // Service call function for pose plan
        bool call_pose_service(const geometry_msgs::Pose& goal_pose, const geometry_msgs::Pose& start_pose, bool is_goal_relative, 
                                trajectory_msgs::JointTrajectory& computed_trajectory, const trajectory_msgs::JointTrajectory& past_trajectory);
        
        // Service call function for slerp plan
        bool call_slerp_service(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory);
        
        // Service call function for joint plan
        bool call_joint_service(std::vector<double> joint_goal, bool flag_state, trajectory_msgs::JointTrajectory& past_trajectory, trajectory_msgs::JointTrajectory& computed_trajectory);
	    
        // Service call function for closing the gripper
        bool call_closing_gripper(bool close);

        // Service call function for opening the gripper
        bool call_opening_gripper(bool open);

        // Service call function for getting the object pose
        bool call_camera_pose(std_msgs::Bool& is_requested_arrived, geometry_msgs::Pose& object_pose);

    /// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Service names   
        std::string arm_control_service_name;
        std::string arm_wait_service_name;
        std::string pose_service_name;
        std::string slerp_service_name;
        std::string joint_service_name;
        std::string gripper_service_grip_in;
        std::string gripper_service_grip_out;

        // Service clients 
        ros::ServiceClient arm_control_client;             // Client for arm control service
        ros::ServiceClient arm_wait_client;                // Client for arm wait service
        ros::ServiceClient pose_client;                    // Client for pose control service
        ros::ServiceClient slerp_client;                    // Client for slerp control service
        ros::ServiceClient joint_client;                    // Client for joint control service
        ros::ServiceClient grip_in_client;
        ros::ServiceClient grip_out_client;
};