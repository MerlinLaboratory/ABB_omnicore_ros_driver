/* POSE CONTROL - Uses moveit movegroupinterface to plan towards a pose */

// Basic includes
#include <ros/service.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

// Custom msg and srv includes
#include "abb_wrapper_msgs/pose_plan.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz
#define     PROMPT   1      // Waits for confermation in RViz before execution
// #define     CONSTRAINTS 1   // Add Joint Constraints (See Path Planning Constraints)

class PosePlan {

    /// public variables and functions ------------------------------------------------------------
	public:
		PosePlan(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_);

        ~PosePlan();

        // This is the callback function of the pose plan service
	  	bool call_pose_plan(abb_wrapper_msgs::pose_plan::Request &req, abb_wrapper_msgs::pose_plan::Response &res);

	  	// Initialize the things for motion planning. It is called by the callback
	  	bool initialize(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, trajectory_msgs::JointTrajectory past_trajectory);

		// Performs motion planning for the end-effector towards goal
		bool performMotionPlan();


	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Important names
        std::string end_effector_name;                          // Name of the end-effector link
        std::string group_name;                                 // Name of the MoveIt group

        // Tf listener and transform and the tmp eigen
	    tf::TransformListener tf_listener;
        tf::StampedTransform stamp_ee_transform;
        Eigen::Affine3d end_effector_state;

        // The goal pose
	  	geometry_msgs::Pose goalPose; 								// The goal pose given by service call
        Eigen::Affine3d goalPoseAff;                                // The same as Eigen Affine
        Eigen::Affine3d startAff;                                   // The staring pose of plan as Eigen Affine

        // Joint trajectory computed to be sent to robot and the past one
        trajectory_msgs::JointTrajectory past_trajectory;
        trajectory_msgs::JointTrajectory computed_trajectory;

        // Boolean which is set by is_reall_null_pose
        bool was_really_null;

        // INLINE PRIVATE FUCTIONS
        // Needed to check if the start pose has been arbitrarily chosen as null pose (this means to plan from present position)
        inline bool is_really_null_pose(geometry_msgs::Pose pose){
            geometry_msgs::Point pos = pose.position;
            geometry_msgs::Quaternion quat = pose.orientation;

            if (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0 &&
                quat.x == 0.0 && quat.y == 0.0 && quat.z == 0.0 && quat.w == 1.0) {
                    return true;
            }
            return false;
        }	
};