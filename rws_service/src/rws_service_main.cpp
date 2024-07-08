#include "../include/rws_service.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rws_service");

    RwsService rws_service = RwsService();
    rws_service.Spinner();
    
    return 0;
}
