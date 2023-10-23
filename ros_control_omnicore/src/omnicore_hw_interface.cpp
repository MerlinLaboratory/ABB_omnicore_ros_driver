#include "../include/omnicore_hw_interface.hpp"

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


namespace ros_control_omnicore
{
   OmnicoreHWInterface::OmnicoreHWInterface(const ros::NodeHandle &nh) : nh(nh)
   {
      // Load the URDF model needs to be loaded
      loadURDF(nh, "robot_description");

      // Load rosparams
      nh.getParam("/robot/name", this->robot_name);
      nh.getParam("/robot/ip_robot", this->ip_robot);
      nh.getParam("/robot/robot_port_rws", this->port_robot_rws);
      nh.getParam("/robot/robot_port_egm", this->port_robot_egm);
      nh.getParam("/robot/name_robot", this->task_name);
      nh.getParam("/robot/task_robot", this->task_robot);
      nh.getParam("/robot/pos_corr_gain", this->pos_corr_gain);
      nh.getParam("/robot/max_speed_deviation", this->max_speed_deviation);
      nh.getParam("/robot_hardware_interface/joints", this->joint_names);

      // Subscribing to topics
      this->omnicore_state_publisher = this->nh.advertise<omnicore_interface::OmnicoreState>("omnicore_state", 1);

      // Publishers to topics with timer
      this->timerOmnicoreState = this->nh.createTimer(ros::Duration(0.5), std::bind(&OmnicoreHWInterface::PublishOmnicoreState, this));

      // Service server instantiation
      this->server_set_egm_state        = this->nh.advertiseService("set_control_to_egm",        &OmnicoreHWInterface::SetControlToEgm,       this);
      this->server_set_free_drive_state = this->nh.advertiseService("set_control_to_free_drive", &OmnicoreHWInterface::SetControlToFreeDrive, this);
      this->server_moveJ_rapid = this->nh.advertiseService("moveJ_rapid", &OmnicoreHWInterface::MoveJRapid, this);
      this->server_moveL_rapid = this->nh.advertiseService("moveL_rapid", &OmnicoreHWInterface::MoveLRapid, this);

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
      this->joint_effort.resize(this->num_joints, 0.0);

      // Commands
      this->joint_position_command.resize(this->num_joints, 0.0);
      this->joint_velocity_command.resize(this->num_joints, 0.0);

      // Limits
      this->joint_position_lower_limits.resize(num_joints, 0.0);
      this->joint_position_upper_limits.resize(num_joints, 0.0);
      this->joint_velocity_limits.resize(num_joints, 0.0);
      this->joint_effort_limits.resize(num_joints, 0.0);

      // Initialize interfaces for each joint
      for (std::size_t joint_id = 0; joint_id < num_joints; joint_id++)
      {
         std::string joint_name = this->joint_names[joint_id];

         // Create joint state interface
         hardware_interface::JointStateHandle joint_handle(this->joint_names[joint_id],
                                                           &this->joint_position[joint_id],
                                                           &this->joint_velocity[joint_id],
                                                           &this->joint_effort[joint_id]);
         this->joint_state_interface.registerHandle(joint_handle);

         // Add command interfaces to joints
         hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(joint_handle, &this->joint_position_command[joint_id]);
         hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(joint_handle, &this->joint_velocity_command[joint_id]);

         this->position_joint_interface.registerHandle(joint_handle_position);
         this->velocity_joint_interface.registerHandle(joint_handle_velocity);

         // Load the joint limits
         registerJointLimits(joint_handle_position, joint_handle_velocity, joint_id);
      }

      registerInterface(&joint_state_interface);    // From RobotHW base class
      registerInterface(&position_joint_interface); // From RobotHW base class
      registerInterface(&velocity_joint_interface); // From RobotHW base class

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

      abb::egm::BaseConfiguration configurations;
      if(this->pos_corr_gain == 0) // 0: velocity control, 1: position control
         configurations.use_velocity_outputs = true;

      this->p_egm_interface = new abb::egm::EGMControllerInterface(this->io_service_, this->port_robot_egm, configurations);
      this->thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));

      // Setting the EGM signal to Start communication, set parameters and wait for response
      if (this->SetEGMParameters() == false)
         ROS_ERROR("Could not send Start signal");
      if (this->EGMStartJointSignal() == false)
         ROS_ERROR("Could not set EGM parameters");
      if (this->p_egm_interface->isInitialized() == false)
         ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
      this->WaitForEgmConnection();


      // --------------------------------------------------------------- //
      // -----------------  Setting ToolData variables ----------------- //
      // --------------------------------------------------------------- //
      this->LoadToolData();

      // Resetting data to send to EGM
      this->reset();

      ROS_INFO_STREAM_NAMED(this->robot_name, "HW Interface Ready!");
   }

   void OmnicoreHWInterface::read(ros::Duration &elapsed_time)
   {
      if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_STOP)
         return;

      // Checking if RAPID StateMachine is running in EGM mode
      if (this->p_egm_interface->waitForMessage(500) == false)
      {
         ROS_ERROR("Timeout for reading data from EGM");
         return;
      }

      // Read the message received from the EGM client
      this->data_from_egm.Clear();
      this->p_egm_interface->read(&data_from_egm);

      int j = 0;
      for (int i = j; i < this->num_joints; i++, j++)
      {
         if (this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
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
      // Checking if RAPID StateMachine is running in IDLE mode. If the current configuration is stream to the robot
      if (this->state_machine_state == abb::rws::RWSStateMachineInterface::STATE_IDLE)
      {
         this->joint_position_command = this->joint_position;
         this->joint_velocity_command = this->joint_velocity;
      }

      // Enforcing limits
      this->pos_jnt_sat_interface.enforceLimits(elapsed_time);

      int j = 0;
      for (int i = j; i < this->num_joints; i++, j++)
      {
         if (!std::isnan(this->joint_position_command[i]) && !std::isinf(this->joint_position_command[i]) &&
             !std::isnan(this->joint_velocity_command[i]) && !std::isinf(this->joint_velocity_command[i]))
         {
            if (this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
            {
               this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->set_values(0, joint_position_command[i] / 3.14159265358979323846 * 180.0);
               this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->set_values(0, joint_velocity_command[i] / 3.14159265358979323846 * 180.0);
               i++;
            }

            this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->set_values(j, joint_position_command[i] / 3.14159265358979323846 * 180.0);
            this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(j, joint_velocity_command[i] / 3.14159265358979323846 * 180.0);
         }
      }

      // ROS_INFO("Position sent: [%lf, %lf, %lf, %lf, %lf, %lf]", joint_position_command[0] / 3.14159265358979323846 * 180.0,
      //                                                           joint_position_command[1] / 3.14159265358979323846 * 180.0,
      //                                                           joint_position_command[2] / 3.14159265358979323846 * 180.0,
      //                                                           joint_position_command[3] / 3.14159265358979323846 * 180.0,
      //                                                           joint_position_command[4] / 3.14159265358979323846 * 180.0,
      //                                                           joint_position_command[5] / 3.14159265358979323846 * 180.0);
      // ROS_INFO("Velocity sent: [%lf, %lf, %lf, %lf, %lf, %lf]", this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[0],
      //                                                           this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[1],
      //                                                           this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[2],
      //                                                           this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[3],
      //                                                           this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[4],
      //                                                           this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->values()[5]);

      // Uncomment to die
      this->p_egm_interface->write(this->data_to_egm);

      return;
   }

   void OmnicoreHWInterface::shutdown()
   {
      std::cout << "Calling shutdown procedure..." << std::endl;

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

   void OmnicoreHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                                 const hardware_interface::JointHandle &joint_handle_velocity,
                                                 std::size_t joint_id)
   {
      // Default values
      this->joint_position_lower_limits[joint_id] = -std::numeric_limits<double>::max();
      this->joint_position_upper_limits[joint_id] = std::numeric_limits<double>::max();
      this->joint_velocity_limits[joint_id] = std::numeric_limits<double>::max();
      this->joint_effort_limits[joint_id] = std::numeric_limits<double>::max();

      // Limits datastructures
      joint_limits_interface::JointLimits joint_limits; // Position
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
      // TODO: It would be better to change this code but I have no idea how to do it
      // It is necessary to set the initial command to send to the robot the actual joint values read from
      // right at this moment. We could try to don't send anything to the robot but I don't know if EGM will complain about it
      // Read the message received from the EGM client
      this->data_from_egm.Clear();
      this->p_egm_interface->read(&this->data_from_egm);

      int j = 0;
      for (int i = j; i < this->num_joints; i++, j++)
      {
         if (this->robot_name == "yumi_single_arm" && i == __EXTERNAL_AXIS__)
         {
            this->joint_position_command[i] = this->data_from_egm.feedback().external().joints().position().values(0); // [Deg]
            this->joint_velocity_command[i] = 0.0;                                                                     // [Deg/s]
            i++;
         }

         this->joint_position_command[i] = this->data_from_egm.feedback().robot().joints().position().values(j); // [Deg]
         this->joint_velocity_command[i] = 0.0;                                                                  // [Deg/s]
      }

      // Setting the space for data to egm
      if (this->robot_name == "yumi_single_arm")
      {
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->Clear();
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->Clear();
      }

      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->Clear();
      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->Clear();

      if (this->robot_name == "yumi_single_arm")
      {
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_position()->CopyFrom(this->data_from_egm.feedback().external().joints().position());
         this->data_to_egm.mutable_external()->mutable_joints()->mutable_velocity()->CopyFrom(this->data_from_egm.feedback().external().joints().velocity());
      }

      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(this->data_from_egm.feedback().robot().joints().position());
      this->data_to_egm.mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(this->data_from_egm.feedback().robot().joints().velocity());

      ROS_INFO("First run after data reset: [%lf, %lf, %lf, %lf, %lf, %lf]", this->joint_position_command[0], this->joint_position_command[1], this->joint_position_command[2],
                                                                             this->joint_position_command[3], this->joint_position_command[4], this->joint_position_command[5]);

      ROS_INFO("Data to EGM have been resetted!");

      return;
   }

   bool OmnicoreHWInterface::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                       const std::list<hardware_interface::ControllerInfo> &stop_list)
   {
      // TODO 
      // Check that the start_list controller constains resources that the Gofa has
      // for(auto eachController : start_list)
      // {
      // 	for(auto eachResource : eachController.claimed_resources.re)
      // 	{
      // 		std::string a = eachResource.resources[0];
      // 	}
      // }
      // Unclaim the resources of the stop_list controllers

      // Check that the claimed resources of start_list controllers exist

      // Deactivate EGM
      ROS_INFO("Calling canSwitch");

      //switch (this->egm_action)
      //{
      //   case abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT:
      //      this->EGMStopJointSignal();
      //      break;
      //   
      //   // case abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_POSE: -> TO BE IMPLEMENTED
      //   //    this->EGMStopPoseSignal();
      //   //    break;

      //   case abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING:
      //      this->EGMStopStreamingSignal();
      //      break;
      //   
      //   default:
      //      ROS_ERROR("Underfined state. Cannot switch controller");
      //      return false;
      //}
      
      return true;
   }

   void OmnicoreHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                      const std::list<hardware_interface::ControllerInfo> &stop_list)
   {
      // TODO 
      // Unclaim the resource of stop_list controllers

      // Claim the resources for start_list controllers

      // Reactivate EGM
      ROS_INFO("Calling doSwitch");
      // this->EGMStartJointSignal();
      
      return;
   }

   // ----------------------------------------------------------------- //
   // --------------- Functions for connecting to Robot --------------- //
   // ----------------------------------------------------------------- //

   bool OmnicoreHWInterface::SetEGMParameters()
   {
      abb::rws::RWSStateMachineInterface::EGMSettings egm_settings;
      if (this->p_rws_interface->services().egm().getSettings(this->task_robot, &egm_settings)) // safer way to apply settings
      {
         egm_settings.activate.max_speed_deviation.value = this->max_speed_deviation; // TODO: Don't know what it means
         egm_settings.allow_egm_motions.value = true;                                 // Brief Flag indicating if EGM motions are allowed to start
         egm_settings.run.pos_corr_gain.value = this->pos_corr_gain;                  // 0=velocity 1=position
         egm_settings.run.cond_time.value = 599.0;                                    // TODO: Don't know what it means
         
         p_rws_interface->requestMasterShip();
         p_rws_interface->services().egm().setSettings(task_robot, egm_settings);
         p_rws_interface->releaseMasterShip();
      }
      else
         return false;

      return true;
   }

   void OmnicoreHWInterface::WaitForEgmConnection()
   {
      bool wait_for_egm_connection = true;
      while (ros::ok() && wait_for_egm_connection)
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
   // --------- Functions for setting the Robot command state --------- //
   // ----------------------------------------------------------------- //

   bool OmnicoreHWInterface::EGMStartJointSignal()
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

   bool OmnicoreHWInterface::EGMStopJointSignal()
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

   bool OmnicoreHWInterface::EGMStartStreamingSignal()
   {
      bool success = this->p_rws_interface->services().egm().signalEGMStartStreaming();

      if(!success)
      {
         ROS_ERROR("Cannot start EGM in streaming mode");
         return success;
      }

      this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;
      this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING;

      return success;
   }

   bool OmnicoreHWInterface::EGMStopStreamingSignal()
   {

      bool success = this->p_rws_interface->services().egm().signalEGMStopStreaming();

      if(!success)
      {
         ROS_ERROR("Cannot stop EGM in streaming mode");
         return success; 
      }

      this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;
      this->egm_action = abb::rws::RWSStateMachineInterface::EGM_ACTION_STOP;

      return success;
   }

   bool OmnicoreHWInterface::FreeDriveStartSignal()
   {
      this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_RUN_RAPID_ROUTINE;

      p_rws_interface->requestMasterShip();
      bool success = this->p_rws_interface->services().rapid().setLeadthroughOn(this->task_robot);
      p_rws_interface->releaseMasterShip();

      if(!success)
         ROS_ERROR("Cannot start robot free drive");

      return success;
   }

   bool OmnicoreHWInterface::FreeDriveStopSignal()
   {
      this->state_machine_state = abb::rws::RWSStateMachineInterface::STATE_IDLE;

      p_rws_interface->requestMasterShip();
      bool success = this->p_rws_interface->services().rapid().setLeadthroughOff(this->task_robot);
      p_rws_interface->releaseMasterShip();

      if(!success)
         ROS_ERROR("Cannot stop robot free drive");

      return success;
   }

   // ----------------------------------------------------------------- //
   // -------------------------- ROS Services ------------------------- //
   // ----------------------------------------------------------------- //

   bool OmnicoreHWInterface::SetControlToEgm(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
   {
      if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_RUN_JOINT)
      {
         ROS_WARN("Robot is already in Egm");
         res.success = true;
         return true;
      }

      // Resetting the joint command to the current one
		ros::Publisher topic_commands = this->nh.advertise<trajectory_msgs::JointTrajectory>("robot_controller/command", 1);
      while (topic_commands.getNumSubscribers() < 1)
      {
         ROS_INFO("Waiting for /command topic...");
         ros::Duration(0.1).sleep();
      }

      trajectory_msgs::JointTrajectory trajectory_msg = trajectory_msgs::JointTrajectory();
      trajectory_msg.joint_names = this->joint_names;
      topic_commands.publish(trajectory_msg);

      // Switching controller from Free Drive to EGM
      res.success = this->EGMStopStreamingSignal();
      ros::Duration(2).sleep();
      res.success = res.success && this->FreeDriveStopSignal();
      ros::Duration(2).sleep();
      res.success = res.success && this->EGMStartJointSignal();

      this->WaitForEgmConnection();

      return true;
   }

   bool OmnicoreHWInterface::SetControlToFreeDrive(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
   {
      if(this->egm_action == abb::rws::RWSStateMachineInterface::EGM_ACTION_STREAMING)
      {
         ROS_WARN("Robot is already in Free Drive");
         res.success = true;
         return true;
      }

      res.success = this->EGMStopJointSignal();
      ros::Duration(2).sleep();
      res.success = res.success && this->FreeDriveStartSignal();
      ros::Duration(2).sleep();
      res.success = res.success && this->EGMStartStreamingSignal();

      return true;
   }

   bool OmnicoreHWInterface::MoveJRapid(omnicore_interface::moveJ_rapidRequest& req, omnicore_interface::moveJ_rapidResponse& res)
   {
      abb::rws::SpeedData speed_data;
      FromVelocityMsgToSpeedData(req.velocity, speed_data);
      res.success = this->p_rws_interface->services().rapid().setMoveSpeed(this->task_robot, speed_data);

      abb::rws::RobTarget robot_target;
      FromGeometryPoseMsgToRobTarget(req.pose, robot_target);

      robot_target.robconf.cf1 = req.configuration[0];
      robot_target.robconf.cf4 = req.configuration[1];
      robot_target.robconf.cf6 = req.configuration[2];
      robot_target.robconf.cfx = req.configuration[3]; // Ignored by 6 axis robots

      res.success = res.success && this->p_rws_interface->services().rapid().runMoveJ(this->task_robot, robot_target);

      if(!res.success)
         ROS_ERROR("Impossible to use MoveJ");

      return true;
   }

   bool OmnicoreHWInterface::MoveLRapid(omnicore_interface::moveL_rapidRequest& req, omnicore_interface::moveL_rapidResponse& res)
   {
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
      res.success = this->p_rws_interface->services().rapid().setMoveSpeed(this->task_robot, speed_data);

      abb::rws::RobTarget robot_target;
      FromGeometryPoseMsgToRobTarget(req.pose, robot_target);

      robot_target.robconf.cf1 = req.configuration[0];
      robot_target.robconf.cf4 = req.configuration[1];
      robot_target.robconf.cf6 = req.configuration[2];
      robot_target.robconf.cfx = req.configuration[3]; // Ignored by 6 axis robots

      res.success = res.success && this->p_rws_interface->services().rapid().runMoveL(this->task_robot, robot_target);

      if(!res.success)
         ROS_ERROR("Impossible to use MoveL");

      return true;
   }

   // ----------------------------------------------------------------- //
   // -------------------- Generic Robot functions -------------------- //
   // ----------------------------------------------------------------- //

   void OmnicoreHWInterface::LoadToolData()
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

      p_rws_interface->releaseMasterShip();
   }

   std_msgs::Byte OmnicoreHWInterface::ReadDigitalInputs()
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

   void OmnicoreHWInterface::PublishOmnicoreState()
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

      // Robot configuration
      omnicoreStateMsg.robot_configuration[0] = std::floor(this->joint_position[0] / (M_PI / 2) );
      omnicoreStateMsg.robot_configuration[1] = std::floor(this->joint_position[3] / (M_PI / 2) );
      omnicoreStateMsg.robot_configuration[2] = std::floor(this->joint_position[5] / (M_PI / 2) );
      
      this->omnicore_state_publisher.publish(omnicoreStateMsg);

      return;
   }

   // ----------------------------------------------------------------- //
   // ------------------------ Debug functions ------------------------ //
   // ----------------------------------------------------------------- //

   void OmnicoreHWInterface::printState()
   {
      // WARNING: THIS IS NOT REALTIME SAFE
      // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
      ROS_INFO_STREAM_THROTTLE(1, std::endl
                                      << printStateHelper());
   }

   std::string OmnicoreHWInterface::printStateHelper()
   {
      std::stringstream ss;
      std::cout.precision(15);

      for (std::size_t i = 0; i < num_joints; ++i)
      {
         ss << "j" << i << ": ";
         ss << std::fixed << this->joint_position[i] * 180 / M_PI << "\t ";
         ss << std::fixed << this->joint_velocity[i] * 180 / M_PI << std::endl;
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

} 
