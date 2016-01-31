/**
 *
 *  \file
 *  \brief      Class representing Hase hardware
 *  \author     Gary Servin <garyservin@gmail.com>
 *  \copyright  Copyright (c) 2015, Gary Servin
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to garyservin@gmail.com
 *
 */

#ifndef HASE_BASE_HASE_HARDWARE_H
#define HASE_BASE_HASE_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "hase_msgs/Drive.h"
#include "hase_msgs/Feedback.h"
#include "hase_msgs/Imu.h"
#include "sensor_msgs/Imu.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


namespace hase_base
{

class HaseHardware : public hardware_interface::RobotHW
{
public:
  HaseHardware();
  void copyJointsFromHardware();
  void publishDriveFromController();

private:
  void feedbackCallback(const hase_msgs::Feedback::ConstPtr& msg);
  void imuCallback(const hase_msgs::Imu::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber imu_sub_;
  realtime_tools::RealtimePublisher<hase_msgs::Drive> cmd_drive_pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::Imu> imu_raw_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  }
  joints_[2];

  // This pointer is set from the ROS thread.
  hase_msgs::Feedback::ConstPtr feedback_msg_;
  boost::mutex feedback_msg_mutex_;
  hase_msgs::Imu::ConstPtr imu_msg_;
  boost::mutex imu_msg_mutex_;
};

}  // namespace hase_base

#endif  // HASE_BASE_HASE_HARDWARE_H
