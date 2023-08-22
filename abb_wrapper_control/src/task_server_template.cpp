/* TASK SERVER THROWING to call throwing task service (DARKO-Stefano)*/
#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"

// Object Includes
#include "abb_wrapper_control/TaskSequencer.h"

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER 
**********************************************/
int main(int argc, char **argv)
{    
    
    ros::init(argc, argv, "task_server_template");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(1);
    spinner.start();
   
   /* 1) Going to Grasp position */
   //Create the request and response object
   while(ros::ok()){
   std_srvs::SetBool::Request req;
   req.data = true;
   std_srvs::SetBool::Response resp;

   ROS_INFO("Going to Grasp position");
   
   bool success_home_position = task_sequencer_obj.call_simple_grasp_task(req,resp);
    
   // Check the success and use of the response

   if(success_home_position){
      ROS_INFO_STREAM("Simple Grasp Service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }
   }
   spinner.stop();

   return 0;
}