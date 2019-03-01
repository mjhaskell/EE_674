#include "autopilot/autopilot.hpp"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "autopilot");
    ros::NodeHandle nh;

    ctrl::Autopilot thing;

    ros::spin();

    return 0;
}
