#include "path_manager/pathmanager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_manager");
    ros::NodeHandle nh;

    PathManager manager_obj;

    ros::spin();

    return 0;
}
