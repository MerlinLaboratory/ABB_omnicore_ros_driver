/* ARM CONTROL - Uses actionlib control the arm
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefanoangeli@ing.unipi.it  */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Custom msg and srv includes
#include "abb_wrapper_msgs/arm_control.h"
#include "abb_wrapper_msgs/arm_wait.h"

// ROS action includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz
// #define     PROMPT          // Waits for confermation in RViz before execution

class ArmControl {

    /// public variables and functions ------------------------------------------------------------
	public:
		ArmControl(ros::NodeHandle& nh_,
            boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_);

        ~ArmControl();
    
        // This is the callback function of the arm control service
	  	bool call_arm_control(abb_wrapper_msgs::arm_control::Request &req, abb_wrapper_msgs::arm_control::Response &res);

		// Sends trajectory to the joint_traj controller
		bool sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory);

        // This is the callback function of the arm wait service
	  	bool call_arm_wait(abb_wrapper_msgs::arm_wait::Request &req, abb_wrapper_msgs::arm_wait::Response &res);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // The arm action client
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr;

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
};