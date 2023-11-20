#ifndef SERVICE_PROVIDER_H
#define SERVICE_PROVIDER_H

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Byte.h>
#include <geometry_msgs/Pose.h>

// ABB libraries
#include <abb_librws/rws_state_machine_interface.h>
#include <abb_librws/rws_interface.h>

// Custom ROS messages
#include <omnicore_interface/OmnicoreState.h>

// Custom ROS services
#include <omnicore_interface/moveJ_rapid.h>
#include <omnicore_interface/moveL_rapid.h>
#include <omnicore_interface/set_digital_output.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

// Boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

class Rws
{
public:
	Rws(const ros::NodeHandle &nh);
	virtual ~Rws() {}

	/** \brief Initialize the rws */
	void init();

	/** \brief Shutdown the rws */
	void shutdown();

private:

	// Ros control functions
	void LoadControllers  (std::vector<std::string>& controllers);
	void UnLoadControllers(std::vector<std::string>& controllers);
	void SwitchControllers(std::vector<std::string>& controllers_to_start,
						   std::vector<std::string>& controllers_to_stop,
						   int strictness,
						   bool start_asap,
						   int timeout);
	std::vector<std::string> GetControllersRunning();

	// EGM functions
	bool EGMSetParams();
	bool EGMStartJointSignal();
	bool EGMStopJointSignal();
	bool EGMStartStreamingSignal();
	bool EGMStopStreamingSignal();

	// FreeDrive functions
	bool FreeDriveStartSignal();
	bool FreeDriveStopSignal();

	// Funcitons to move from one command to the other
	bool SetControlToEgmSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
	bool SetControlToFreeDriveSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
	bool MoveJRapidSrv(omnicore_interface::moveJ_rapidRequest& req, omnicore_interface::moveJ_rapidResponse& res);
	bool MoveLRapidSrv(omnicore_interface::moveL_rapidRequest& req, omnicore_interface::moveL_rapidResponse& res);
	bool ShutdownSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
	bool SetDigitalOutputSrv(omnicore_interface::set_digital_outputRequest& req, omnicore_interface::set_digital_outputResponse& res);

	// Generic funtions
	void 		   LoadToolData();
	std_msgs::Byte ReadDigitalInputs();
	bool		   SetDigitalOutput(uint8_t index, uint8_t value);
	void 		   PublishOmnicoreState();

	std::string robot_name;
	std::string task_name;
	ros::NodeHandle nh;

	// Robot configuration
	std::vector<std::string> joint_names;
	std::size_t num_joints;

	// --------------------------------------------------------------- //
	// ---------------------- Variables for ROS ---------------------- //
	// --------------------------------------------------------------- //
	
	// Publishers
	ros::Timer timerOmnicoreState;
	ros::Publisher omnicore_state_publisher;

	// Ros services servers
	ros::ServiceServer server_set_egm_state;
	ros::ServiceServer server_set_free_drive_state;
	ros::ServiceServer server_moveJ_rapid;
	ros::ServiceServer server_moveL_rapid;
	ros::ServiceServer server_set_egm_params;
	ros::ServiceServer server_egm_shutdown;
	ros::ServiceServer server_set_digital_output;

	// Ros services clients
	ros::ServiceClient client_load_controllers;
	ros::ServiceClient client_unload_controllers;
	ros::ServiceClient client_switch_controllers;
	ros::ServiceClient client_list_controllers;

	// --------------------------------------------------------------- //
	// -------------- Variables for connecting to Robot -------------- //
	// --------------------------------------------------------------- //

	// Generic data
	std::string ip_robot;
	int port_robot_rws;
	int port_robot_egm;
	std::string task_robot;

	// Ros_controllers running
	std::vector<std::string> controllers_running_before_free_drive;

	// StateMachine state
	abb::rws::RWSStateMachineInterface::States state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;
	abb::rws::RWSStateMachineInterface::EGMActions egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_UNKNOWN;

	// EGM parameters
	int pos_corr_gain;
	int max_speed_deviation;
	
	// Rws interface
	abb::rws::RWSStateMachineInterface* p_rws_interface;
};

#endif 