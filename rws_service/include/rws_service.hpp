
// ROS
#include <ros/ros.h>

// ABB libraries
#include <abb_librws/rws_state_machine_interface.h>
#include <abb_librws/rws_interface.h>
#include <abb_librws/rws_rapid.h>

// Including services
#include <rws_service/GripIn.h>
#include <rws_service/GripOut.h>
#include <rws_service/MoveTo.h>

#include <iostream>



class RwsService
{
private:

public:

	RwsService();

    // ---------------- Methods ---------------- //

    // Services Callbacks
    bool GripInSrv (rws_service::GripIn:: Request &req, rws_service::GripIn ::Response &res);
    bool GripOutSrv(rws_service::GripOut::Request &req, rws_service::GripOut::Response &res);
    bool MoveToSrv (rws_service::MoveTo:: Request &req, rws_service::MoveTo ::Response &res);

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