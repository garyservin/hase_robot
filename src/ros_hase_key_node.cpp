#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist vel;

    vel.angular.z = 0.0;
    vel.linear.x = 1.0 ;

    //ROS_INFO("%", .data.c_str());
    ROS_DEBUG("[i] forward %.2f %.2f", vel.linear.x, vel.angular.z);

    vel_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
