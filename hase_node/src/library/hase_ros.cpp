#include "hase_node/hase_ros.hpp"

namespace hase
{

/*****************************************************************************
 ** Implementation [HaseRos]
 *****************************************************************************/

HaseRos::HaseRos()
{
}

/**
 * This will wait some time while kobuki internally closes its threads and destructs
 * itself.
 */
HaseRos::~HaseRos()
{
  ROS_INFO_STREAM("Hase : waiting for hase thread to finish.");
}

bool HaseRos::init(ros::NodeHandle& nh)
{
  /*********************
   ** Communications
   **********************/
  subscribeTopics(nh);

  hase.init();
  hase.enable();
  return true;
}
/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void HaseRos::subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe(std::string("cmd_vel"), 10, &HaseRos::subscribeVelocityCommand, this);
}


} // namespace hase

