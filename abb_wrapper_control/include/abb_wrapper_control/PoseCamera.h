/* POSE CAMERA - Get grasp pose for the dice w.r.t to single_yumi_base_link
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <geometry_msgs/PoseStamped.h>

// Custom msg and srv includes
#include "abb_wrapper_msgs/pose_camera.h"

class PoseCamera{

    /// public variables and functions ------------------------------------------------------------
	public:
		PoseCamera(ros::NodeHandle& nh_);

        ~PoseCamera();

		// Callback for the object pose subscriber

        void get_object_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
        // This is the callback function of the joint plan service
	  	bool call_pose_camera(abb_wrapper_msgs::pose_camera::Request &req, abb_wrapper_msgs::pose_camera::Response &res);
        //
		bool isPoseEmpty(const geometry_msgs::PoseStamped& object_pose);

	/// private variables -------------------------------------------------------------------------
	private:
	    //
		ros::NodeHandle nh;
	    // Subscriber to object pose and the pose
        ros::Subscriber object_sub;
		geometry_msgs::PoseStamped object_pose_T;
	    std::string object_topic_name = "/camera/grasp_pose";
};