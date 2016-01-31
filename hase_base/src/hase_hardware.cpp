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

#include <boost/assign.hpp>
#include "hase_base/hase_hardware.h"

namespace hase_base
{

HaseHardware::HaseHardware()
{
  ros::V_string joint_names = boost::assign::list_of("left_wheel")
      ("right_wheel");

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("feedback", 1, &HaseHardware::feedbackCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);

  imu_sub_ = nh_.subscribe("imu/data_hase", 1, &HaseHardware::imuCallback, this);
  imu_raw_pub_.init(nh_, "imu/data_raw", 1);
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void HaseHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 2; i++)
    {
      joints_[i].position = feedback_msg_->drivers[i].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i].measured_velocity;
      joints_[i].effort = 0;
    }
  }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void HaseHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    cmd_drive_pub_.msg_.drivers[hase_msgs::Drive::LEFT] = joints_[0].velocity_command;
    cmd_drive_pub_.msg_.drivers[hase_msgs::Drive::RIGHT] = joints_[1].velocity_command;
    cmd_drive_pub_.unlockAndPublish();
  }
}

void HaseHardware::feedbackCallback(const hase_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

void HaseHardware::imuCallback(const hase_msgs::Imu::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(imu_msg_mutex_);

  if (imu_raw_pub_.trylock())
  {
    // Fill header
    imu_raw_pub_.msg_.header = msg->header;
    imu_raw_pub_.msg_.header.frame_id = "imu";

    // Fill quaternion orientation
    imu_raw_pub_.msg_.orientation.x = 0.0;
    imu_raw_pub_.msg_.orientation.y = 0.0;
    imu_raw_pub_.msg_.orientation.z = 0.0;
    imu_raw_pub_.msg_.orientation.w = 0.0;
    imu_raw_pub_.msg_.orientation_covariance[0] = 0.0003;
    imu_raw_pub_.msg_.orientation_covariance[4] = 0.0003;
    imu_raw_pub_.msg_.orientation_covariance[8] = 0.0003;

    // Fill angular velocities
    imu_raw_pub_.msg_.angular_velocity = msg->angular_velocity;
    imu_raw_pub_.msg_.angular_velocity_covariance[0] = 0.0003;
    imu_raw_pub_.msg_.angular_velocity_covariance[4] = 0.0003;
    imu_raw_pub_.msg_.angular_velocity_covariance[8] = 0.0003;

    // Fill linear accelerations
    imu_raw_pub_.msg_.linear_acceleration = msg->linear_acceleration;
    imu_raw_pub_.msg_.linear_acceleration_covariance[0] = 0.0003;
    imu_raw_pub_.msg_.linear_acceleration_covariance[4] = 0.0003;
    imu_raw_pub_.msg_.linear_acceleration_covariance[8] = 0.0003;

    imu_raw_pub_.unlockAndPublish();
  }
}

}  // namespace hase_base
