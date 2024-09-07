#include "../include/rws_services.hpp"

RwsService::RwsService()
{
    // Reading config file
    nh.getParam("/robot/ip_robot"      , this->ip_robot      );
    nh.getParam("/robot/robot_port_rws", this->port_robot_rws);
    float max_force = 200;
    float max_speed = 120;
    if(nh.hasParam("/robot/smart_gripper/max_force") && nh.hasParam("/robot/smart_gripper/max_force"))
    {
        nh.getParam("/robot/smart_gripper/max_force", max_force);
        nh.getParam("/robot/smart_gripper/max_speed", max_speed);
    }

    // Enstablishing connection with RWS server
    const Poco::Net::Context::Ptr ptrContext(new Poco::Net::Context(Poco::Net::Context::CLIENT_USE, "", "", "", Poco::Net::Context::VERIFY_NONE));
	p_rws_interface = new abb::rws::RWSStateMachineInterface(this->ip_robot, this->port_robot_rws, ptrContext);

    // Create the required server services
    server_grip_in  = this->nh.advertiseService("grip_in",  &RwsService::GripInSrv , this);
    server_grip_out = this->nh.advertiseService("grip_out", &RwsService::GripOutSrv, this);
    server_move_to  = this->nh.advertiseService("move_gripper_to",  &RwsService::MoveToSrv , this);
    this->p_rws_interface->services().sg().Calibrate((uint)max_force,(uint)max_speed);
}

bool RwsService::GripInSrv(std_srvs::Trigger::Request  &req, 
                           std_srvs::Trigger::Response &res)
{
    res.success = this->p_rws_interface->services().sg().GripIn();

    return true;
}

bool RwsService::GripOutSrv(std_srvs::Trigger::Request  &req, 
                            std_srvs::Trigger::Response &res)
{
    res.success = this->p_rws_interface->services().sg().GripOut();
    return true;
}

bool RwsService::MoveToSrv(omnicore_interface::move_gripper_to::Request  &req, 
                           omnicore_interface::move_gripper_to::Response &res)
{
    p_rws_interface->requestMasterShip();
    usleep(250000);
    abb::rws::RAPIDNum temp_command(abb::rws::RWSStateMachineInterface::SGCommands::SG_COMMAND_MOVE_TO);
    res.success = this->p_rws_interface->services().sg().MoveTo(req.position);
    p_rws_interface->releaseMasterShip();
    usleep(250000);

    return true;
}

void RwsService::Spinner()
{
    ros::spin();
}
