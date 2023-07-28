#include <time.h>
#include <controller_manager/controller_manager.h>
#include "omnicore_hw_interface.hpp"

namespace ros_control_omnicore
{
  // Used to convert seconds elapsed to nanoseconds
  static const double BILLION = 1000000000.0;

  /**
   * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a
   * specified frequency
   *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
   *        See
   * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
   */
  class OmnicoreHWControlLoop
  {
    public:
      /**
       * \brief Constructor
       * \param NodeHandle
       * \param hardware_interface - the robot-specific hardware interface to be use with your robot
       */
      OmnicoreHWControlLoop(ros::NodeHandle &nh, std::shared_ptr<ros_control_omnicore::OmnicoreHWInterface> omnicore_hardware_interface);

      // Run the control loop (blocking)
      void run();

    private:
      // Update funcion called with loop_hz_ rate
      void update();

      // Startup and shutdown of the internal node inside a roscpp program
      ros::NodeHandle nh_;

      // Name of this class
      std::string name_ = "omnicore_hw_control_loop";

      // Settings
      ros::Duration desired_update_period_;
      double cycle_time_error_threshold_;

      // Timing
      double loop_hz_;
      struct timespec last_time_;

      /** \brief ROS Controller Manager and Runner
       *
       * This class advertises a ROS interface for loading, unloading, starting, and
       * stopping ros_control-based controllers. It also serializes execution of all
       * running controllers in \ref update.
       */
      std::shared_ptr<controller_manager::ControllerManager> controller_manager;

      /** \brief Abstract Hardware Interface for your robot */
      std::shared_ptr<ros_control_omnicore::OmnicoreHWInterface> hardware_interface;

  }; // end class

} // namespace ros_control_omnicore
