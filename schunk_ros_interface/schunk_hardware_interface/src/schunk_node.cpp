#include <string>
#include <iostream>

#include "../include/schunk_eip_interface.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "schunk_gripper");

    std::shared_ptr<SchunkGripper> p_gripper = std::make_shared<SchunkGripper>();

    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin();
    ros::shutdown();
    
    return 0;
}