#include <../include/gofa_hw_interface.hpp>

namespace ros_control_gofa
{
   GofaHWInterface::GofaHWInterface(const ros::NodeHandle &nh): name("Gofa_hw_interface"), nh(nh) 
   {
      // Load the URDF model needs to be loaded
      loadURDF(nh, "robot_description");

      // Load rosparams
      ros::NodeHandle rpnh(nh, "hardware_interface"); 
      std::size_t error = 0;
      error += !rosparam_shortcuts::get(this->name, rpnh, "joints", this->joint_names);
      rosparam_shortcuts::shutdownIfError(this->name, error);
   }

   // ! TODO: check if the Gofa supports all these interfaces
   void GofaHWInterface::init()
   {
      this->num_joints = this->joint_names.size();

      // Status
      this->joint_position.resize(this->num_joints, 0.0);
      this->joint_velocity.resize(this->num_joints, 0.0);
      this->joint_effort  .resize(this->num_joints, 0.0);

      // Commands
      this->joint_position_command.resize(this->num_joints, 0.0);
      this->joint_velocity_command.resize(this->num_joints, 0.0);
      this->joint_effort_command  .resize(this->num_joints, 0.0);

      // Limits
      this->joint_position_lower_limits.resize(num_joints, 0.0);
      this->joint_position_upper_limits.resize(num_joints, 0.0);
      this->joint_velocity_limits      .resize(num_joints, 0.0);
      this->joint_effort_limits        .resize(num_joints, 0.0);

      // Initialize interfaces for each joint
      for (std::size_t joint_id = 0; joint_id < num_joints; joint_id++)
      {
         std::string joint_name = this->joint_names[joint_id];
         ROS_DEBUG_STREAM_NAMED(this->name, "Loading joint name: " << joint_name);

         // Create joint state interface
         hardware_interface::JointStateHandle joint_handle(this->joint_names    [joint_id], 
                                                           &this->joint_position[joint_id], 
                                                           &this->joint_velocity[joint_id], 
                                                           &this->joint_effort  [joint_id]);
         this->joint_state_interface.registerHandle(joint_handle);

         // Add command interfaces to joints
         hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(joint_handle, &this->joint_position_command[joint_id]);
         hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(joint_handle, &this->joint_velocity_command[joint_id]);
         hardware_interface::JointHandle joint_handle_effort   = hardware_interface::JointHandle(joint_handle, &this->joint_effort_command  [joint_id]);
         
         this->position_joint_interface.registerHandle(joint_handle_position);
         this->velocity_joint_interface.registerHandle(joint_handle_velocity);
         this->effort_joint_interface  .registerHandle(joint_handle_effort);

         // Load the joint limits
         registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
      }

      registerInterface(&joint_state_interface);    // From RobotHW base class
      registerInterface(&position_joint_interface); // From RobotHW base class
      registerInterface(&velocity_joint_interface); // From RobotHW base class
      registerInterface(&effort_joint_interface);   // From RobotHW base class
      
      // ! 
      // ! TODO: Initialise connection with Gofa EGM interface
      // ! 

      ROS_INFO_STREAM_NAMED(this->name, "GofaHWInterface Ready.");
   }

   void GofaHWInterface::read(ros::Duration &elapsed_time)
   {
      return;
   }

   void GofaHWInterface::write(ros::Duration &elapsed_time)
   {
      return;
   }

   void GofaHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id)
   {
      // Default values
      this->joint_position_lower_limits[joint_id] = -std::numeric_limits<double>::max();
      this->joint_position_upper_limits[joint_id] = std::numeric_limits<double>::max();
      this->joint_velocity_limits      [joint_id] = std::numeric_limits<double>::max();
      this->joint_effort_limits        [joint_id] = std::numeric_limits<double>::max();

      // Limits datastructures
      joint_limits_interface::JointLimits joint_limits;    // Position
      joint_limits_interface::SoftJointLimits soft_limits; // Soft Position
      bool has_joint_limits = false;
      bool has_soft_limits = false;

      // Get limits from URDF
      if (this->urdf_model == NULL)
      {
         ROS_WARN_STREAM_NAMED(this->name, "No URDF model loaded, unable to get joint limits");
         return;
      }

      // Get limits from URDF
      std::string joint_name = this->joint_names[joint_id];

      urdf::JointConstSharedPtr urdf_joint = this->urdf_model->getJoint(joint_name);
      if (urdf_joint == NULL || joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
      {
         has_joint_limits = true;
         ROS_DEBUG_STREAM_NAMED(this->name, "Joint " << joint_name << " has URDF position limits ["
                                                     << joint_limits.min_position << ", " 
                                                     << joint_limits.max_position << "]");

         // Setting the position limits
         // Slighly reduce the joint limits to prevent floating point errors
         joint_limits.min_position += std::numeric_limits<double>::epsilon();
         joint_limits.max_position -= std::numeric_limits<double>::epsilon();
         this->joint_position_lower_limits[joint_id] = joint_limits.min_position;
         this->joint_position_upper_limits[joint_id] = joint_limits.max_position;
         
         // Setting the velocity limits
         if (joint_limits.has_velocity_limits)
         {
            ROS_DEBUG_STREAM_NAMED(this->name, "Joint " << joint_name << " has URDF velocity limit ["
                                                        << joint_limits.max_velocity << "]");
            this->joint_velocity_limits[joint_id] = joint_limits.max_velocity;
         }

         // Setting effort limits if available
         if (joint_limits.has_effort_limits)
         {
            ROS_DEBUG_STREAM_NAMED(this->name, "Joint " << joint_name << " has URDF effor limit ["
                                             << joint_limits.max_effort << "]");
            this->joint_effort_limits[joint_id] = joint_limits.max_effort;
         }
      }
      else
      {
         ROS_WARN_STREAM_NAMED(this->name, "Joint " << joint_name << " does not have a URDF position limit");
         return;
      }

      // For soft joint limit use saturation limits
      ROS_DEBUG_STREAM_NAMED(this->name, "Using saturation limits (not soft limits)");

      const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
      const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
      const joint_limits_interface::EffortJointSaturationHandle   sat_handle_effort  (joint_handle_effort  , joint_limits);

      pos_jnt_sat_interface.registerHandle(sat_handle_position);
      vel_jnt_sat_interface.registerHandle(sat_handle_velocity);
      eff_jnt_sat_interface.registerHandle(sat_handle_effort);

   }

   void GofaHWInterface::reset()
   {
      // Reset joint limits state, in case of mode switch or e-stop
      this->pos_jnt_sat_interface.reset();
      // this->pos_jnt_soft_limits.reset();
   }

   // ----------------------------------------------------------------- // 
   // ------------------------ Debug functions ------------------------ // 
   // ----------------------------------------------------------------- // 

   void GofaHWInterface::printState()
   {
      // WARNING: THIS IS NOT REALTIME SAFE
      // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
      ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
   }


   std::string GofaHWInterface::printStateHelper()
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

   std::string GofaHWInterface::printCommandHelper()
   {
      std::stringstream ss;
      std::cout.precision(15);
      ss << "    position     velocity         effort  \n";
      for (std::size_t i = 0; i < num_joints; ++i)
      {
         ss << "j" << i << ": ";
         ss << std::fixed << this->joint_position_command[i] << "\t ";
         ss << std::fixed << this->joint_velocity_command[i] << "\t ";
         ss << std::fixed << this->joint_effort_command  [i] << std::endl;
      }
      return ss.str();
   }

   void GofaHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
   {
      std::string urdf_string;
      this->urdf_model = new urdf::Model();

      // search and wait for robot_description on param server
      while (urdf_string.empty() && ros::ok())
      {
         std::string search_param_name;
         if (nh.searchParam(param_name, search_param_name))
         {
            ROS_INFO_STREAM_NAMED(this->name, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                             << search_param_name);
            nh.getParam(search_param_name, urdf_string);
         }
         else
         {
            ROS_INFO_STREAM_NAMED(this->name, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                                             << param_name);
            nh.getParam(param_name, urdf_string);
         }

         usleep(100000);
      }

      if (!this->urdf_model->initString(urdf_string))
         ROS_ERROR_STREAM_NAMED(this->name, "Unable to load URDF model");
      else
         ROS_DEBUG_STREAM_NAMED(this->name, "Received URDF from param server");
   }

} // namespace ros_control_gofa
