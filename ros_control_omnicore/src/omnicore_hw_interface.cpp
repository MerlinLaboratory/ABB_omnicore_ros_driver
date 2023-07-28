#include "../include/omnicore_hw_interface.hpp"

namespace ros_control_omnicore
{
   OmnicoreHWInterface::OmnicoreHWInterface(const ros::NodeHandle &nh): nh(nh) 
   {
      // Load the URDF model needs to be loaded
      loadURDF(nh, "robot_description");

      // Load rosparams

      // Loading parameters from configuration.yaml
      nh.getParam("/robot/name"                     , this->robot_name         );
      nh.getParam("/robot/ip_robot"                 , this->ip_robot           );
      nh.getParam("/robot/robot_port_rws"           , this->port_robot_rws     );
      nh.getParam("/robot/robot_port_egm"           , this->port_robot_egm     );
      nh.getParam("/robot/name_robot"               , this->task_name          );
      nh.getParam("/robot/task_robot"               , this->task_robot         );
      nh.getParam("/robot/pos_corr_gain"            , this->pos_corr_gain      );
      nh.getParam("/robot/max_speed_deviation"      , this->max_speed_deviation);
      nh.getParam("/robot_hardware_interface/joints", this->joint_names        );

      // Debug
	   ROS_INFO("Robot: %s has n_joints: %ld", this->robot_name.c_str(), this->joint_names.size());
	   ROS_INFO("For RWS, connecting to ip: %s and port: %s", this->ip_robot.c_str(), std::to_string(this->port_robot_rws).c_str());
	   ROS_INFO("For EGM, port: %s", std::to_string(this->port_robot_egm).c_str());
   }

   void OmnicoreHWInterface::init()
   {
      this->num_joints = this->joint_names.size();

      // Status
      this->joint_position.resize(this->num_joints, 0.0);
      this->joint_velocity.resize(this->num_joints, 0.0);
      this->joint_effort  .resize(this->num_joints, 0.0);

      // Commands
      this->joint_position_command.resize(this->num_joints, 0.0);
      this->joint_velocity_command.resize(this->num_joints, 0.0);
      // this->joint_effort_command  .resize(this->num_joints, 0.0); -> not supported by Omnicore :()

      // Limits
      this->joint_position_lower_limits.resize(num_joints, 0.0);
      this->joint_position_upper_limits.resize(num_joints, 0.0);
      this->joint_velocity_limits      .resize(num_joints, 0.0);
      this->joint_effort_limits        .resize(num_joints, 0.0);

      // Initialize interfaces for each joint
      for (std::size_t joint_id = 0; joint_id < num_joints; joint_id++)
      {
         std::string joint_name = this->joint_names[joint_id];
         // ROS_INFO("Loading joint name: %s", joint_name.c_str());

         // Create joint state interface
         hardware_interface::JointStateHandle joint_handle(this->joint_names    [joint_id], 
                                                           &this->joint_position[joint_id], 
                                                           &this->joint_velocity[joint_id],
                                                           &this->joint_effort  [joint_id]);
         this->joint_state_interface.registerHandle(joint_handle);

         // Add command interfaces to joints
         hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(joint_handle, &this->joint_position_command[joint_id]);
         hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(joint_handle, &this->joint_velocity_command[joint_id]);
         // hardware_interface::JointHandle joint_handle_effort   = hardware_interface::JointHandle(joint_handle, &this->joint_effort_command  [joint_id]); -> Not supported by Omnicore :(

         this->position_joint_interface.registerHandle(joint_handle_position);
         this->velocity_joint_interface.registerHandle(joint_handle_velocity);
         // this->effort_joint_interface  .registerHandle(joint_handle_effort); -> Not supported by Omnicore :(

         // Load the joint limits
         registerJointLimits(joint_handle_position, joint_handle_velocity, joint_id);
      }

      registerInterface(&joint_state_interface);    // From RobotHW base class
      registerInterface(&position_joint_interface); // From RobotHW base class
      registerInterface(&velocity_joint_interface); // From RobotHW base class
      // registerInterface(&effort_joint_interface);   // From RobotHW base class -> Not supported by Omnicore :(

      // --------------------------------------------------------------- //
      // ----------  Ensablishing connection with RWS and EGM ---------- //
      // --------------------------------------------------------------- //
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

      this->p_egm_interface = new abb::egm::EGMControllerInterface(io_service_, this->port_robot_egm);
		this->thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
      
      // Setting the EGM signal to Start communication, set parameters and wait for response
      if(this->EGMStartSignal() == false)
         ROS_ERROR("Could not send Start signal");
      if( this->SetEGMParameters() == false )
         ROS_ERROR("Could not set EGM parameters");
		if (this->p_egm_interface->isInitialized() == false)
			ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
      this->WaitForEgmConnection();

      // TODO: It would be better to change this code but I have no idea how to do it
      // It is necessary to set the initial command to send to the robot the actual joint values read from 
      // right at this moment. We could try to don't send anything to the robot but I don't know if EGm will complain about it
      // Read the message received from the EGM client
      this->data_from_egm.Clear();
      this->p_egm_interface->read(&this->data_from_egm);

      int j = 0;
      for(int i = j; i < this->num_joints; i++, j++)
      {
         if(this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
         {
            this->joint_position_command[i] = this->data_from_egm.feedback().external().joints().position().values(0); // [Deg]
            this->joint_velocity_command[i] = 0.0;                                                                     // [Deg/s]
            i++;
         }

         this->joint_position_command[i] = this->data_from_egm.feedback().robot().joints().position().values(j); // [Deg]
         this->joint_velocity_command[i] = 0.0;                                                                  // [Deg/s]
      } 

      // Setting the space for data to egm
      if(this->robot_name == "yumi_single_arm")
      {
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->Clear();
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->Clear();
      }
      
      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->Clear();
      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->Clear();

      // TODO: verify that they are useless 
		// this->data_to_egm.mutable_robot()->mutable_joints()->Clear();
		// this->data_to_egm.mutable_robot()->Clear();
		// this->data_to_egm.Clear();

      if(this->robot_name == "yumi_single_arm")
      {
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->CopyFrom(this->data_from_egm.feedback().external().joints().position());
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->CopyFrom(this->data_from_egm.feedback().external().joints().velocity());
      }

      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(this->data_from_egm.feedback().robot().joints().position());
      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(this->data_from_egm.feedback().robot().joints().velocity());

      ROS_INFO_STREAM_NAMED(this->robot_name, "HW Interface Ready!");
   }

   void OmnicoreHWInterface::read(ros::Duration &elapsed_time)
   {
      if (this->p_egm_interface->waitForMessage(500)  == false)
      {
         ROS_ERROR("Timeout for reading");
         return;
      }

      // Read the message received from the EGM client
      this->data_from_egm.Clear();
      this->p_egm_interface->read(&data_from_egm);

      int j = 0;
      for(int i = j; i < this->num_joints; i++, j++)
      {
         if(this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
         {
            this->joint_position[i] = this->data_from_egm.feedback().external().joints().position().values(0) * 3.14159265358979323846 / 180.0; // [rad]
            this->joint_velocity[i] = this->data_from_egm.feedback().external().joints().velocity().values(0) * 3.14159265358979323846 / 180.0; // [rad/s]
            i++;
         }

         this->joint_position[i] = this->data_from_egm.feedback().robot().joints().position().values(j) * 3.14159265358979323846 / 180.0; // [rad]
         this->joint_velocity[i] = this->data_from_egm.feedback().robot().joints().velocity().values(j) * 3.14159265358979323846 / 180.0; // [rad/s]
      }

      return;
   }

   void OmnicoreHWInterface::write(ros::Duration &elapsed_time)
   {
      // Enforcing limits
      this->pos_jnt_sat_interface.enforceLimits(elapsed_time);
      
      int j = 0;
      for(int i = j; i < this->num_joints; i++, j++)
      {
		   if (!std::isnan(this->joint_position_command[i]) && !std::isinf(std::isnan(this->joint_position_command[i])) &&
             !std::isnan(this->joint_velocity_command[i]) && !std::isinf(std::isnan(this->joint_velocity_command[i])) )
         {
            if(this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
            {
               this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->set_values(0, joint_position_command[i] / 3.14159265358979323846 * 180.0);
               this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->set_values(0, joint_velocity_command[i] / 3.14159265358979323846 * 180.0);
               i++;
            }

            this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->set_values(j, joint_position_command[i] / 3.14159265358979323846 * 180.0);
            this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(j, joint_velocity_command[i] / 3.14159265358979323846 * 180.0);
         }
      }

      // ROS_INFO("Joints: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", joint_position_command[0] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[1] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[2] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[3] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[4] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[5] / 3.14159265358979323846 * 180.0,
      //                                                         joint_position_command[6] / 3.14159265358979323846 * 180.0);

      // Uncomment to die
		this->p_egm_interface->write(this->data_to_egm);

      return;
   }

   void OmnicoreHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             std::size_t joint_id)
   {
      // Default values
      this->joint_position_lower_limits[joint_id] = -std::numeric_limits<double>::max();
      this->joint_position_upper_limits[joint_id] =  std::numeric_limits<double>::max();
      this->joint_velocity_limits      [joint_id] =  std::numeric_limits<double>::max();
      this->joint_effort_limits        [joint_id] =  std::numeric_limits<double>::max();

      // Limits datastructures
      joint_limits_interface::JointLimits joint_limits;    // Position
      bool has_joint_limits = false;

      // Get limits from URDF
      if (this->urdf_model == NULL)
      {
         ROS_WARN_STREAM_NAMED(this->robot_name, "No URDF model loaded, unable to get joint limits");
         return;
      }

      // Get limits from URDF
      std::string joint_name = this->joint_names[joint_id];
      urdf::JointConstSharedPtr urdf_joint = this->urdf_model->getJoint(joint_name);

      if (urdf_joint != NULL && joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
      {
         has_joint_limits = true;
         // ROS_INFO("Joint %s has URDF position limits [ %lf, %lf]", joint_name.c_str(), joint_limits.min_position, joint_limits.max_position);

         // Setting the position limits
         // Slighly reduce the joint limits to prevent floating point errors
         joint_limits.min_position += std::numeric_limits<double>::epsilon();
         joint_limits.max_position -= std::numeric_limits<double>::epsilon();
         this->joint_position_lower_limits[joint_id] = joint_limits.min_position;
         this->joint_position_upper_limits[joint_id] = joint_limits.max_position;
         
         // Setting the velocity limits
         if (joint_limits.has_velocity_limits)
         {
            // ROS_INFO("Joint %s has max URDF velocity limit: %lf]", joint_name.c_str(), joint_limits.max_velocity);
            this->joint_velocity_limits[joint_id] = joint_limits.max_velocity;
         }
      }
      else
      {
         ROS_WARN_STREAM_NAMED(this->robot_name, "Joint " << joint_name << " does not have a URDF limits");
         return;
      }

      // For soft joint limit use saturation limits
      ROS_DEBUG_STREAM_NAMED(this->robot_name, "Using saturation limits (not soft limits)");

      const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
      const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);

      pos_jnt_sat_interface.registerHandle(sat_handle_position);
      vel_jnt_sat_interface.registerHandle(sat_handle_velocity);
   }

   void OmnicoreHWInterface::reset()
   {
      // Reset joint limits state, in case of mode switch or e-stop
      this->pos_jnt_sat_interface.reset();
   }

   // ----------------------------------------------------------------- //
   // --------------- Functions for connecting to Robot --------------- //
   // ----------------------------------------------------------------- //

   bool OmnicoreHWInterface::SetEGMParameters()
   {
	   abb::rws::RWSStateMachineInterface::EGMSettings egm_settings;
      if (this->p_rws_interface->services().egm().getSettings(this->task_robot, &egm_settings)) // safer way to apply settings
      {
         p_rws_interface->requestMasterShip();

         egm_settings.activate.max_speed_deviation.value = this->max_speed_deviation; // TODO: Don't know what it means
         egm_settings.allow_egm_motions.value            = true;                      // Brief Flag indicating if EGM motions are allowed to start
         egm_settings.run.pos_corr_gain.value            = this->pos_corr_gain;       // 0=velocity 1=position
         egm_settings.run.cond_time.value                = 599.0;                     // TODO: Don't know what it means

         p_rws_interface->services().egm().setSettings(task_robot, egm_settings);
         p_rws_interface->releaseMasterShip();
      }
      else
         return false;
      
      return true;
   }

   bool OmnicoreHWInterface::EGMStartSignal()
   {
      return this->p_rws_interface->services().egm().signalEGMStartJoint();

		//if(this->p_rws_interface->pulseIOSignal("EGM_START_JOINT", 500000))
      //   return true;
      
      //ROS_ERROR("Cannot trigger Signal EGM_START_JOINT");
      //return false;
   }

   void OmnicoreHWInterface::WaitForEgmConnection()
   {
      bool wait_for_egm_connection = true;
      while(ros::ok() && wait_for_egm_connection)
      {
         if (this->p_egm_interface->isConnected())
			{
				if (this->p_egm_interface->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
					ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
				else
					wait_for_egm_connection = this->p_egm_interface->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
			}
			else
				ROS_INFO("EGM is NOT CONNECTED");
      }
   }

   // ----------------------------------------------------------------- // 
   // ------------------------ Debug functions ------------------------ // 
   // ----------------------------------------------------------------- // 

   void OmnicoreHWInterface::printState()
   {
      // WARNING: THIS IS NOT REALTIME SAFE
      // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
      ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
   }

   std::string OmnicoreHWInterface::printStateHelper()
   {
      std::stringstream ss;
      std::cout.precision(15);

      for (std::size_t i = 0; i < num_joints; ++i)
      {
         ss << "j" << i << ": ";
         ss << std::fixed << this->joint_position[i] << "\t ";
         ss << std::fixed << this->joint_velocity[i] << "\t ";
         ss << std::fixed << this->joint_effort  [i] << std::endl;
      }
      return ss.str();
   }

   std::string OmnicoreHWInterface::printCommandHelper()
   {
      std::stringstream ss;
      std::cout.precision(15);
      ss << "    position     velocity         effort  \n";
      for (std::size_t i = 0; i < num_joints; ++i)
      {
         ss << "j" << i << ": ";
         ss << std::fixed << this->joint_position_command[i] << "\t ";
         ss << std::fixed << this->joint_velocity_command[i] << std::endl;
      }
      return ss.str();
   }

   void OmnicoreHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
   {
      std::string urdf_string;
      this->urdf_model = new urdf::Model();

      // search and wait for robot_description on param server
      while (urdf_string.empty() && ros::ok())
      {
         std::string search_param_name;
         if (nh.searchParam(param_name, search_param_name))
         {
            ROS_INFO_STREAM_NAMED(this->robot_name, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                             << search_param_name);
            nh.getParam(search_param_name, urdf_string);
         }
         else
         {
            ROS_INFO_STREAM_NAMED(this->robot_name, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                             << param_name);
            nh.getParam(param_name, urdf_string);
         }

         usleep(100000);
      }

      if (!this->urdf_model->initString(urdf_string))
         ROS_ERROR_STREAM_NAMED(this->robot_name, "Unable to load URDF model");
      else
         ROS_INFO("Received URDF from param server");
   }

} // namespace ros_control_omnicore
