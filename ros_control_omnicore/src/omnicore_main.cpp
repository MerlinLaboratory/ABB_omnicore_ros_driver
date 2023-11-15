#include "../include/omnicore_control_loop.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omnicore_hw_interface");
	ros::NodeHandle nh;

	// NOTE: We run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(3);
	spinner.start();

	// Create the hardware interface specific to your robot
	std::shared_ptr<ros_control_omnicore::OmnicoreHWInterface> p_hw_interface = std::make_shared<ros_control_omnicore::OmnicoreHWInterface>(nh);
	p_hw_interface->init();

	// Create the control loop
	ros_control_omnicore::OmnicoreControlLoop control_loop(nh, p_hw_interface);

	control_loop.run(); // Blocks until shutdown signal recieved

	return 0;
}
