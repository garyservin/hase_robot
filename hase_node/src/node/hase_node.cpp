#include "hase_node/hase_ros.hpp"

hase::HaseRos hase_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hase_node");
    ros::NodeHandle n;
    hase_.init(n);

    while (ros::ok()) {
        ros::spin();
    }

    ROS_INFO("End");
    return 0;
}
