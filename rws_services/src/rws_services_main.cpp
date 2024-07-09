#include "../include/rws_services.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rws_services");

    RwsService rws_service = RwsService();
    rws_service.Spinner();
    
    return 0;
}
