#include "hase_node/hase_ros.hpp"

namespace hase
{

/*****************************************************************************
 ** Implementation [HaseRos]
 *****************************************************************************/

HaseRos::HaseRos()
{
  ROS_INFO_STREAM("Hase : Created object.");
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
  nh.param("use_bbb", usebbb_, false);
  /*********************
   ** Communications
   **********************/
  subscribeTopics(nh);
  advertiseTopics(nh);

  if(usebbb_){
  #ifdef USE_BBB
    hase.init();
    hase.enable();
  #endif
  }else{
    // Do something for HaseMbed
  }

  return true;
}
/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kobuki esoterics.
 */
void HaseRos::advertiseTopics(ros::NodeHandle& nh)
{
  if(!usebbb_){
    twist_mbed_publisher = nh.advertise < geometry_msgs::Twist > ("hase/cmd_vel", 100);
  }
}

void HaseRos::subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe(std::string("cmd_vel"), 10, &HaseRos::subscribeVelocityCommand, this);
}

} // namespace hase

