#ifndef HASE_ROS_HPP_
#define HASE_ROS_HPP_

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <hase_driver/hase.hpp>
#include <hase_firmware/hase_mbed.hpp>

namespace hase
{
class HaseRos
{
public:
  HaseRos();
  ~HaseRos();
  bool init(ros::NodeHandle& nh);

private:
  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  #ifdef USE_BBB
  Hase hase;
  #endif
  HaseMbed hase_mbed;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool usebbb_; // If we're using the Beaglebone Black
  /*********************
   ** Ros Comms
   **********************/
  ros::Publisher version_info_publisher;
  ros::Publisher twist_mbed_publisher;

  ros::Subscriber velocity_command_subscriber;

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);

  // debugging
  void rosDebug(const std::string &msg) { ROS_DEBUG_STREAM("Hase : " << msg); }
  void rosInfo(const std::string &msg) { ROS_INFO_STREAM("Hase : " << msg); }
  void rosWarn(const std::string &msg) { ROS_WARN_STREAM("Hase : " << msg); }
  void rosError(const std::string &msg) { ROS_ERROR_STREAM("Hase : " << msg); }
  void rosNamed(const std::vector<std::string> &msgs) {
    if (msgs.size()==0) return;
    if (msgs.size()==1) { ROS_INFO_STREAM("Hase : " << msgs[0]); }
    if (msgs.size()==2) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM("Hase : " << msgs[1]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM ("Hase : " << msgs[1]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM ("Hase : " << msgs[1]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM("Hase : " << msgs[1]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM("Hase : " << msgs[1]); }
    }
    if (msgs.size()==3) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM_NAMED(msgs[1], "Hase : " << msgs[2]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM_NAMED (msgs[1], "Hase : " << msgs[2]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM_NAMED (msgs[1], "Hase : " << msgs[2]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM_NAMED(msgs[1], "Hase : " << msgs[2]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM_NAMED(msgs[1], "Hase : " << msgs[2]); }
    }
  }
};

} // namespace hase

#endif /* HASE_ROS_HPP_ */
