#include "../include/rws.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omnicore_rws");
	ros::NodeHandle nh;

	// Create the rws specific to your robot
	std::shared_ptr<Rws> p_rws = std::make_shared<Rws>(nh);
	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	p_rws->shutdown();

	return 0;
}
