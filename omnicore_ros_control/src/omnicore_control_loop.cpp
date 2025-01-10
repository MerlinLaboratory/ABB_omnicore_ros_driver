#include "../include/omnicore_control_loop.hpp"

namespace ros_control_omnicore
{
	OmnicoreControlLoop::OmnicoreControlLoop(ros::NodeHandle &nh, std::shared_ptr<ros_control_omnicore::OmnicoreHWInterface> hardware_interface)
		 : nh_(nh), hardware_interface(hardware_interface)
	{
		// Create the controller manager
		this->controller_manager.reset(new controller_manager::ControllerManager(hardware_interface.get(), nh_));

		// Load rosparams
		nh.getParam("/robot_hw_control_loop/loop_hz", this->loop_hz_);
		nh.getParam("/robot_hw_control_loop/cycle_time_error_threshold", this->cycle_time_error_threshold_);

		// Get current time for use with first update
		clock_gettime(CLOCK_MONOTONIC, &last_time_);

		this->desired_update_period_ = ros::Duration(1 / loop_hz_);

		this->server_shutdown_control_loop_srv = this->nh_.advertiseService("shutdown_control_loop", &OmnicoreControlLoop::shutdownControlLoopSrv, this);
	}

	void OmnicoreControlLoop::run()
	{
		ros::Rate rate(loop_hz_);
		while (ros::ok())
		{
			if(this->shutdown)
				ros::shutdown();
			else
			{
				update();
				rate.sleep();
			}
		}		
	}

	void OmnicoreControlLoop::update()
	{
		// Get change in time
		struct timespec current_time_;
		clock_gettime(CLOCK_MONOTONIC, &current_time_);
		ros::Duration elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
		this->last_time_ = current_time_;
		ros::Time now = ros::Time::now();

		// Error check cycle time
		const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
		if (cycle_time_error > cycle_time_error_threshold_)
		{
			ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
														<< cycle_time_error << ", cycle time: " << elapsed_time_
														<< ", threshold: " << cycle_time_error_threshold_);
		}

		// Input
		this->hardware_interface->read(elapsed_time_);

		// Control
		this->controller_manager->update(now, elapsed_time_);

		// Output
		this->hardware_interface->write(elapsed_time_);
	}

	bool OmnicoreControlLoop::shutdownControlLoopSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
	{
		this->shutdown = true;
		this->hardware_interface->shutdown();
		ROS_INFO("hardware interface shutdown");
		res.success = true;
		return true;
	}

} // namespace ros_control_omnicore
