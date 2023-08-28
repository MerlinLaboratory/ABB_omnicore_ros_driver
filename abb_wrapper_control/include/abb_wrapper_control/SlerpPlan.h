/* SLERP CONTROL - Uses SLERP interpolation between present ee pose and goal pose to create homogeneous motion
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it  */

// Basic includes
#include <ros/service.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Custom msg and srv includes
#include "abb_wrapper_msgs/slerp_plan.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz
#define     PROMPT  1        // Waits for confermation in RViz before execution

class SlerpPlan {

    /// public variables and functions ------------------------------------------------------------
	public:
		SlerpPlan(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_, int n_wp_);

        ~SlerpPlan();

        // This is the callback function of the slerp plan service
	  	bool call_slerp_plan(abb_wrapper_msgs::slerp_plan::Request &req, abb_wrapper_msgs::slerp_plan::Response &res);

	  	// Initialize the things for motion planning. It is called by the callback
	  	bool initialize(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, trajectory_msgs::JointTrajectory past_trajectory);

		// Performs motion planning for the end-effector towards goal
		bool performMotionPlan();

		// Computes waypoints using SLERP from two poses
		void computeWaypointsFromPoses(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& goal_pose, std::vector<geometry_msgs::Pose>& waypoints);


	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Important names
        std::string end_effector_name;                          // Name of the end-effector link
        std::string group_name;                                 // Name of the MoveIt group

        // Number of waypoints for slerp interpolation
        int n_wp;                                               // Number of wp to be used for a translation of 1 meter
        int real_n_wp;                                          // Number of wp for requested motion (proportional to n_wp)

        // Tf listener and transform and the tmp eigen
	    tf::TransformListener tf_listener;
        tf::StampedTransform stamp_ee_transform;
        Eigen::Affine3d end_effector_state;

        // The start and goal poses of the control
	  	Eigen::Affine3d startAff; 								// The starting ee pose 
	  	Eigen::Affine3d goalAff; 								// The goal pose given by service call

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