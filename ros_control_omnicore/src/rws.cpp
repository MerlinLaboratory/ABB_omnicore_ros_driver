#include "../include/rws.hpp"

void FromGeometryPoseMsgToRobTarget(geometry_msgs::Pose pose, abb::rws::RobTarget& robot_target)
{
   robot_target.pos.x = pose.position.x;
   robot_target.pos.y = pose.position.y;
   robot_target.pos.z = pose.position.z;
   robot_target.orient.q1 = pose.orientation.w;
   robot_target.orient.q2 = pose.orientation.x;
   robot_target.orient.q3 = pose.orientation.y;
   robot_target.orient.q4 = pose.orientation.z;

   return;
}

void FromVelocityMsgToSpeedData(uint8_t velocity, abb::rws::SpeedData& speed_data)
{
   speed_data.v_ori  = 500 ;
   speed_data.v_leax = 5000;
   speed_data.v_reax = 1000;

   switch (velocity)
   {
      case omnicore_interface::moveJ_rapid::Request::V50 :
         speed_data.v_tcp = 50;
         break;

      case omnicore_interface::moveJ_rapid::Request::V100:
         speed_data.v_tcp = 100;
         break;

      case omnicore_interface::moveJ_rapid::Request::V300:
         speed_data.v_tcp = 300;
         break;

      case omnicore_interface::moveJ_rapid::Request::V500:
         speed_data.v_tcp = 500;
         break;
      case omnicore_interface::moveJ_rapid::Request::V1000:
         speed_data.v_tcp = 1000;
         break;      
   
      default:
         break;
   }

   return;
}

Rws::Rws(const ros::NodeHandle &nh) : nh(nh)
{
	// Load rosparams
	nh.getParam("/robot/name"					, this->robot_name	  	   );
	nh.getParam("/robot/ip_robot"				, this->ip_robot	  	   );
	nh.getParam("/robot/robot_port_rws"			, this->port_robot_rws	   );
	nh.getParam("/robot/name_robot"				, this->task_name	  	   );
	nh.getParam("/robot/task_robot"				, this->task_robot	  	   );
   	nh.getParam("/robot/pos_corr_gain"			, this->pos_corr_gain	   );
    nh.getParam("/robot/max_speed_deviation"	, this->max_speed_deviation);

	// Subscribing to topics
	this->omnicore_state_publisher = this->nh.advertise<omnicore_interface::OmnicoreState>("omnicore_state", 1);

	// Publishers to topics with timer
	this->timerOmnicoreState = this->nh.createTimer(ros::Duration(0.5), std::bind(&Rws::PublishOmnicoreState, this));

	// Service server instantiation
	this->server_set_egm_state        = this->nh.advertiseService("set_control_to_egm",        &Rws::SetControlToEgmSrv,       this);
	this->server_set_free_drive_state = this->nh.advertiseService("set_control_to_free_drive", &Rws::SetControlToFreeDriveSrv, this);
	this->server_egm_shutdown		  = this->nh.advertiseService("egm_shutdown"	         , &Rws::ShutdownSrv, 		       this);
	this->server_moveJ_rapid 		  = this->nh.advertiseService("moveJ_rapid"				 , &Rws::MoveJRapidSrv, 		   this);
	this->server_moveL_rapid 		  = this->nh.advertiseService("moveL_rapid"				 , &Rws::MoveLRapidSrv, 		   this);
	this->server_set_digital_output	  = this->nh.advertiseService("set_digital_output"	     , &Rws::SetDigitalOutputSrv, 	   this);

	// Debug
	ROS_INFO("For RWS, connecting to ip: %s and port: %s", this->ip_robot.c_str(), std::to_string(this->port_robot_rws).c_str());

	// ------------------------------------------------------- //
	// ----------  Ensablishing connection with RWS ---------- //
	// ------------------------------------------------------- //
	const Poco::Net::Context::Ptr ptrContext(new Poco::Net::Context(Poco::Net::Context::CLIENT_USE, "", "", "", Poco::Net::Context::VERIFY_NONE));
	this->p_rws_interface = new abb::rws::RWSStateMachineInterface(this->ip_robot, this->port_robot_rws, ptrContext);

	this->p_rws_interface->stopRAPIDExecution();
	usleep(250000);
	this->p_rws_interface->requestMasterShip();
	usleep(250000);
	this->p_rws_interface->resetRAPIDProgramPointer();
	usleep(250000);
	this->p_rws_interface->releaseMasterShip();
	usleep(250000);
	this->p_rws_interface->startRAPIDExecution();
	usleep(250000);

	// Setting params + control to EGM
	if(EGMSetParams() == false)
		ROS_ERROR("Cannot load EGM params");
	if(this->EGMStartJointSignal() == false)
	{
		ROS_ERROR("Cannot put robot in EGM mode");
		this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT;
	}

	// Service client instantiation
    // this->client_load_controllers    = this->nh.serviceClient<controller_manager_msgs::LoadController>  ("/controller_manager/load_controller");
    this->client_unload_controllers  = this->nh.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
    this->client_switch_controllers  = this->nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    this->client_list_controllers    = this->nh.serviceClient<controller_manager_msgs::ListControllers> ("/controller_manager/list_controllers");

	while(this->client_unload_controllers.waitForExistence(ros::Duration(1))   == false || 
		  this->client_switch_controllers.waitForExistence(ros::Duration(1)) == false ||
		  this->client_list_controllers.waitForExistence(ros::Duration(1)) == false)
	{
		ROS_INFO("Waiting for controller_manger services");
	}

	// --------------------------------------------------------------- //
	// -----------------  Setting ToolData variables ----------------- //
	// --------------------------------------------------------------- //
	this->LoadToolData();


	ROS_INFO_STREAM_NAMED(this->robot_name, "Rws Ready!");
}

void Rws::shutdown()
{
	std::cout << "Calling shutdown procedure..." << std::endl;

	// Unloading all controllers
	std::cout << "Unoading ros controllers ..." << std::endl;
	
	std::vector<std::string> controllers_runnning = this->GetControllersRunning();
	this->UnLoadControllers(controllers_runnning);

	std::cout << "Killing EGM ..." << std::endl;
	switch (this->state_machine_state)
	{
		case abb::rws::RWSStateMachineInterface::STATE_IDLE:
		{
			/* Do nothing. You are already in a safe state */
			break;
		}
		case abb::rws::RWSStateMachineInterface::STATE_RUN_EGM_ROUTINE:
		{
			/* Exit from EGM state */
			this->EGMStopJointSignal();
			break;
		}
		case abb::rws::RWSStateMachineInterface::STATE_RUN_RAPID_ROUTINE:
		{
			/* It should do something but I don't know what*/
			break;
		}
		default:
			ROS_ERROR("Statemachine State not found! Unpredictable robot behaviours could happen!");
	}

	ros::shutdown();

	return;
}

// ----------------------------------------------------------------- //
// --------- Functions for setting the Robot command state --------- //
// ----------------------------------------------------------------- //

bool Rws::EGMStartJointSignal()
{
	bool success = this->p_rws_interface->services().egm().signalEGMStartJoint();

	if(!success)
	{
		ROS_ERROR("Cannot start EGM in joint mode");
		return success;
	}

	this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_RUN_EGM_ROUTINE;
	this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT;

	return success;
}

bool Rws::EGMStopJointSignal()
{
	bool success = this->p_rws_interface->services().egm().signalEGMStop();

	if(!success)
	{
		ROS_ERROR("Cannot stop EGM in joint mode");
		return success; 
	}

	this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;
	this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_STOP;

	return success;
}

bool Rws::EGMStartStreamingSignal()
{
	bool success = this->p_rws_interface->services().egm().signalEGMStartStreaming();

	if(!success)
	{
		ROS_ERROR("Cannot start EGM in streaming mode");
		return success;
	}

	this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING;

	return success;
}

bool Rws::EGMStopStreamingSignal()
{
	bool success = this->p_rws_interface->services().egm().signalEGMStopStreaming();

	if(!success)
	{
		ROS_ERROR("Cannot stop EGM in streaming mode");
		return success; 
	}

	this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_STOP;

	return success;
}

bool Rws::FreeDriveStartSignal()
{
	p_rws_interface->requestMasterShip();
	bool success = this->p_rws_interface->services().rapid().setLeadthroughOn(this->task_robot);
	p_rws_interface->releaseMasterShip();

	if(!success)
		ROS_ERROR("Cannot start robot free drive");
	else
	{
		nh.setParam("/robot/is_free_drive_on", true);
		this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_FREE_DRIVE_ROUTINE;
	}

	return success;
}

bool Rws::FreeDriveStopSignal()
{
	this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;

	p_rws_interface->requestMasterShip();
	bool success = this->p_rws_interface->services().rapid().setLeadthroughOff(this->task_robot);
	p_rws_interface->releaseMasterShip();

	if(!success)
		ROS_ERROR("Cannot stop robot free drive");
	{
		nh.setParam("/robot/is_free_drive_on", false);
		this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;
	}

	return success;
}

// ----------------------------------------------------------------- //
// -------------------------- ROS Services ------------------------- //
// ----------------------------------------------------------------- //

bool Rws::SetControlToEgmSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
	if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT || 
	   this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_POSE)
	{
		ROS_WARN("Robot is already in Egm");
		res.success = true;
		return true;
	}

	// Calling service SwitchController
	std::vector<std::string> controllers_to_start = this->controllers_running_before_free_drive;
	std::vector<std::string> controllers_to_stop  = {};
	this->SwitchControllers(controllers_to_start, controllers_to_stop, 2, false, 0.0);

	// Stopping streaming if active
	if( this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING)
	{
		res.success = this->EGMStopStreamingSignal();
		ros::Duration(2).sleep();
	}

	if(this->state_machine_state == abb::rws::RWSStateMachineInterface::STATE_FREE_DRIVE_ROUTINE)
	{
		res.success = res.success && this->FreeDriveStopSignal();
		ros::Duration(2).sleep();
	}

	res.success = res.success && this->EGMStartJointSignal();

	return true;
}

bool Rws::SetControlToFreeDriveSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
	if(this->state_machine_state == abb::rws::RWSStateMachineInterface::STATE_FREE_DRIVE_ROUTINE)
	{
		ROS_WARN("Robot is already in Free Drive");
		res.success = true;
		return true;
	}

	// Getting all running controllers
	std::vector<std::string> controllers_running = GetControllersRunning();

	// Check whether you are starting/stopping joint_state_cotroller
	this->controllers_running_before_free_drive.clear();
	for(std::string controller : controllers_running)
	{
		if(controller != "joint_state_controller")
			this->controllers_running_before_free_drive.push_back(controller);
	}

	// Calling service UnloadController
	std::vector<std::string> controllers_to_start = {};
	std::vector<std::string> controllers_to_stop  = controllers_running_before_free_drive;
	this->SwitchControllers(controllers_to_start, controllers_to_stop, 2, false, 0.0);

	if(this->egm_action != abb::rws::RWSStateMachineInterface::EGM_ACTION_STOP)
	{
		res.success = this->EGMStopJointSignal();
		ros::Duration(2).sleep();
	}

	res.success = res.success && this->FreeDriveStartSignal();
	ros::Duration(2).sleep();

	if(this->egm_action != abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING)
		res.success = res.success && this->EGMStartStreamingSignal();

	return true;
}

bool Rws::MoveJRapidSrv(omnicore_interface::moveJ_rapidRequest& req, omnicore_interface::moveJ_rapidResponse& res)
{
	res.success = true; 

	// Checking first that the robot is in free drive
	if(this->state_machine_state == abb::rws::RWSStateMachineInterface::STATE_FREE_DRIVE_ROUTINE)
	{
		res.success = res.success && this->FreeDriveStopSignal();
		ros::Duration(2).sleep();
	}

	// Checking first that the robot is not controlled in EGM
	if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT || 
	   this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_POSE)
	{
		res.success = res.success && this->EGMStopJointSignal();
		ros::Duration(2).sleep();
		res.success = res.success && this->EGMStartStreamingSignal();
		ros::Duration(2).sleep();
	}

	abb::rws::SpeedData speed_data;
	FromVelocityMsgToSpeedData(req.velocity, speed_data);
	p_rws_interface->requestMasterShip();
	res.success = this->p_rws_interface->services().rapid().setMoveSpeed(this->task_robot, speed_data);

	abb::rws::RobTarget robot_target;
	FromGeometryPoseMsgToRobTarget(req.pose, robot_target);

	robot_target.robconf.cf1 = req.configuration[0];
	robot_target.robconf.cf4 = req.configuration[1];
	robot_target.robconf.cf6 = req.configuration[2];
	robot_target.robconf.cfx = req.configuration[3]; // Ignored by 6 axis robots

	res.success = res.success && this->p_rws_interface->services().rapid().runMoveJ(this->task_robot, robot_target);
	p_rws_interface->releaseMasterShip();

	if(!res.success)
		ROS_ERROR("Impossible to use MoveJ");

	return true;
}

bool Rws::MoveLRapidSrv(omnicore_interface::moveL_rapidRequest& req, omnicore_interface::moveL_rapidResponse& res)
{
	res.success = true; 

	// Checking first that the robot is in free drive
	if(this->state_machine_state == abb::rws::RWSStateMachineInterface::STATE_FREE_DRIVE_ROUTINE)
	{
		res.success = res.success && this->FreeDriveStopSignal();
		ros::Duration(2).sleep();
	}

	// Checking first that the robot is not controlled in EGM
	if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT || 
		this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_POSE)
	{
		res.success = this->EGMStopJointSignal();
		ros::Duration(2).sleep();
		res.success = res.success && this->EGMStartStreamingSignal();
		ros::Duration(2).sleep();
	}

	abb::rws::SpeedData speed_data;
	FromVelocityMsgToSpeedData(req.velocity, speed_data);

	p_rws_interface->requestMasterShip();
	res.success = this->p_rws_interface->services().rapid().setMoveSpeed(this->task_robot, speed_data);

	abb::rws::RobTarget robot_target;
	FromGeometryPoseMsgToRobTarget(req.pose, robot_target);

	robot_target.robconf.cf1 = req.configuration[0];
	robot_target.robconf.cf4 = req.configuration[1];
	robot_target.robconf.cf6 = req.configuration[2];
	robot_target.robconf.cfx = req.configuration[3]; // Ignored by 6 axis robots

	res.success = res.success && this->p_rws_interface->services().rapid().runMoveL(this->task_robot, robot_target);
	p_rws_interface->releaseMasterShip();

	if(!res.success)
		ROS_ERROR("Impossible to use MoveL");


	return true;
}

bool Rws::EGMSetParams()
{
	abb::rws::RWSStateMachineInterface::EGMSettings egm_settings;
	if (this->p_rws_interface->services().egm().getSettings(this->task_robot, &egm_settings)) // safer way to apply settings
	{
		egm_settings.activate.max_speed_deviation.value = this->max_speed_deviation;
		egm_settings.allow_egm_motions.value 		    = true;                 
		egm_settings.run.pos_corr_gain.value 			= this->pos_corr_gain;  
		egm_settings.run.cond_time.value 				= 599.0;                    
		
		p_rws_interface->requestMasterShip();
		p_rws_interface->services().egm().setSettings(task_robot, egm_settings);
		p_rws_interface->releaseMasterShip();
	}
	else
	{
		ROS_ERROR("Cannot set EGM params");
		return false;
	}

	return true;
}

bool Rws::ShutdownSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
	this->shutdown();
	return true;
}

bool Rws::SetDigitalOutputSrv(omnicore_interface::set_digital_outputRequest& req, omnicore_interface::set_digital_outputResponse& res)
{
	res.success = this->SetDigitalOutput(req.port, req.value);
	return true;
}

// ----------------------------------------------------------------- //
// -------------------- Generic Robot functions -------------------- //
// ----------------------------------------------------------------- //

void Rws::LoadToolData()
{
	std::vector<float> tcp_pose;
	float mass;
	std::vector<float> cog;
	std::vector<float> inertia_tensor;
	std::vector<float> aom;

	nh.getParam("/robot/tool_data/tcp_pose"      , tcp_pose      );
	nh.getParam("/robot/tool_data/mass"          , mass          );
	nh.getParam("/robot/tool_data/cog"           , cog           );
	nh.getParam("/robot/tool_data/inertia_tensor", inertia_tensor);
	nh.getParam("/robot/tool_data/aom"           , aom           );

	p_rws_interface->requestMasterShip();
	abb::rws::ToolData tool_data;
	tool_data.robhold.value = true;

	// Geometry data
	tool_data.tframe.pos.x  = tcp_pose[0]; 
	tool_data.tframe.pos.y  = tcp_pose[1]; 
	tool_data.tframe.pos.z  = tcp_pose[2]; 
	tool_data.tframe.rot.q1 = tcp_pose[3];
	tool_data.tframe.rot.q2 = tcp_pose[4];
	tool_data.tframe.rot.q3 = tcp_pose[5];
	tool_data.tframe.rot.q4 = tcp_pose[6];

	// Load data
	tool_data.tload.mass   = mass;
	tool_data.tload.cog.x  = cog[0]; 
	tool_data.tload.cog.y  = cog[1]; 
	tool_data.tload.cog.z  = cog[2]; 
	tool_data.tload.ix     = inertia_tensor[0];
	tool_data.tload.iy     = inertia_tensor[1];
	tool_data.tload.iz     = inertia_tensor[2];
	tool_data.tload.aom.q1 = aom[0];
	tool_data.tload.aom.q2 = aom[1];
	tool_data.tload.aom.q3 = aom[2];
	tool_data.tload.aom.q4 = aom[3];

	bool success = this->p_rws_interface->services().rapid().setCurrentToolData(this->task_robot, tool_data);

	if(!success)
	{
		ROS_ERROR("Impossible to set tool_data! Killing application");
		ros::shutdown();
	}
	else
		ROS_INFO("Tool_data loaded successfully");

	p_rws_interface->releaseMasterShip();
}

std_msgs::Byte Rws::ReadDigitalInputs()
{     
	std_msgs::Byte digital_input_status = std_msgs::Byte();

	p_rws_interface->requestMasterShip();

	uint number_controller_digital_inputs_ports = abb::rws::SystemConstants::IOSignals::OmnicoreDigitalInputs.size();
	for(int port = 0; port < number_controller_digital_inputs_ports; port++)
	{
		std::string port_name = abb::rws::SystemConstants::IOSignals::OmnicoreDigitalInputs[port];
		std::string port_status_string = this->p_rws_interface->getIOSignal(port_name);

		// 0x00 = 00000000
		// 0x01 = 00000001 
		char port_status_int = port_status_string == abb::rws::SystemConstants::IOSignals::HIGH ? 0x01 : 0x00;

		digital_input_status.data = digital_input_status.data | ( port_status_int << port );
	}
	p_rws_interface->releaseMasterShip();

	return digital_input_status;
}

bool Rws::SetDigitalOutput(uint8_t port, uint8_t value)
{
	const unsigned int number_controller_digital_output_ports = abb::rws::SystemConstants::IOSignals::OmnicoreDigitalOutputs.size();
	if(port < 1 || port > number_controller_digital_output_ports)
	{
		ROS_ERROR("Digital Output port is out of bounds. Expected 1-8");
		return false;
	}

	this->p_rws_interface->requestMasterShip();

	std::string port_string  = abb::rws::SystemConstants::IOSignals::OmnicoreDigitalOutputs[port - 1];
	std::string value_string = (value > 0) ? abb::rws::SystemConstants::IOSignals::HIGH : abb::rws::SystemConstants::IOSignals::LOW;
	bool success = this->p_rws_interface->setIOSignal(port_string, value_string);

	if(success == false)
		ROS_INFO("Fail to set port %s to value %s", port_string.c_str(), value_string.c_str());

	this->p_rws_interface->releaseMasterShip();
	return success;
}

void Rws::PublishOmnicoreState()
{
	omnicore_interface::OmnicoreState omnicoreStateMsg = omnicore_interface::OmnicoreState();

	// Controller state
	switch (this->egm_action)
	{
		case abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT: 
			omnicoreStateMsg.controller_state = omnicore_interface::OmnicoreState::EGM_MODE;
			break;
		
		case abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING: 
			omnicoreStateMsg.controller_state = omnicore_interface::OmnicoreState::FREE_DRIVE_MODE;
			break;

		default:
			break;
	}

	// Digital inputs
	omnicoreStateMsg.digital_inputs_state = ReadDigitalInputs().data;

	// Robot configuration // TODO: fix configuration since you don't have any way to know joint values 
	// omnicoreStateMsg.robot_configuration[0] = std::floor(this->joint_position[0] / (M_PI / 2) );
	// omnicoreStateMsg.robot_configuration[1] = std::floor(this->joint_position[3] / (M_PI / 2) );
	// omnicoreStateMsg.robot_configuration[2] = std::floor(this->joint_position[5] / (M_PI / 2) );
	
	this->omnicore_state_publisher.publish(omnicoreStateMsg);

	return;
}

// --------------------------------------------------------------- //
// -------------------- Ros control functions -------------------- //
// --------------------------------------------------------------- //

void Rws::UnLoadControllers(std::vector<std::string>& controllers)
{
	for(std::string controller : controllers)
	{
		controller_manager_msgs::UnloadController::Request  req;
		controller_manager_msgs::UnloadController::Response res;

		req.name = controller;
		this->client_unload_controllers.call(req, res);

		if(!res.ok)
			ROS_ERROR("Impossible to unload %s", controller.c_str());
		else
			ROS_INFO("Controller %s unloaded successfully", controller.c_str());
	}
}

void Rws::SwitchControllers(std::vector<std::string>& controllers_to_start,
						std::vector<std::string>& controllers_to_stop,
						int strictness,
						bool start_asap,
						int timeout)
{
	controller_manager_msgs::SwitchController::Request  req;
	controller_manager_msgs::SwitchController::Response res;

	req.start_controllers = controllers_to_start;
	req.stop_controllers = controllers_to_stop;
	req.start_asap = start_asap;
	req.strictness = req.STRICT;
	req.timeout = 0.0;

	this->client_switch_controllers.call(req, res);

	if(!res.ok)
		ROS_ERROR("Impossible to switch controllers");

}

std::vector<std::string> Rws::GetControllersRunning()
{
	std::vector<std::string> controllers_running;

	controller_manager_msgs::ListControllers::Request  req;
	controller_manager_msgs::ListControllers::Response res;

	this->client_list_controllers.call(req, res);

	for(auto controller : res.controller)
	{
		if(controller.state == "running")
		{
			ROS_INFO("Controller: %s is in state: %s", controller.name.c_str(), controller.state.c_str());
			controllers_running.push_back(controller.name);
		}
	}

	return controllers_running;
}