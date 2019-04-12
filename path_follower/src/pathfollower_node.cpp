#include "path_follower/pathfollower.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;

    PathFollower follower_obj;

    ros::spin();

    return 0;
}
