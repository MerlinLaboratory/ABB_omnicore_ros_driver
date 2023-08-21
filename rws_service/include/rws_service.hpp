
// ROS
#include <ros/ros.h>

// ABB libraries
#include <abb_librws/rws_state_machine_interface.h>
#include <abb_librws/rws_interface.h>
#include <abb_librws/rws_rapid.h>

// Including services
#include "std_srvs/Trigger.h"

#include <iostream>



class RwsService
{
private:

public:

	RwsService();

    // ---------------- Methods ---------------- //

    // Services Callbacks
    bool GripInSrv (std_srvs::Trigger:: Request &req, std_srvs::Trigger::Response &res);
    bool GripOutSrv(std_srvs::Trigger::Request &req,  std_srvs::Trigger::Response &res);
    // bool MoveToSrv (std_srvs::Trigger:: Request &req, std_srvs::Trigger::Response &res);

    void Spinner();

    // ---------------- Variables ---------------- //

    ros::NodeHandle nh;

    // Robot parameters
    std::string ip_robot;      
    int port_robot_rws;

    // Services
    ros::ServiceServer server_grip_in;
    ros::ServiceServer server_grip_out;
    ros::ServiceServer server_move_to;


	abb::rws::RWSStateMachineInterface* p_rws_interface;

};