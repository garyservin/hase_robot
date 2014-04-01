/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Marcus Liebhardt
 *
 * This work has been inspired by Nate Koenig's Gazebo plugin for the iRobot Create.
 */

#include <cmath>
#include <cstring>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <gazebo/math/gzmath.hh>
#include "hase_gazebo_plugins/gazebo_ros_hase.h"

namespace gazebo
{

enum {LEFT= 0, RIGHT=1};

GazeboRosHase::GazeboRosHase() : shutdown_requested_(false)
{
  motors_enabled_ = true;
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Initialise variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
}

GazeboRosHase::~GazeboRosHase()
{
  shutdown_requested_ = true;
}

void GazeboRosHase::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;
  if (!model_)
  {
    ROS_ERROR_STREAM("Invalid model pointer! [" << node_name_ << "]");
    return;
  }
  // Get then name of the parent model and use it as node name
  std::string model_name = sdf->GetParent()->Get<std::string>("name");
  gzdbg << "Plugin model name: " << model_name << "\n";
  nh_ = ros::NodeHandle("");
  // creating a private name pace until Gazebo implements topic remappings
  nh_priv_ = ros::NodeHandle("/" + model_name);
  node_name_ = model_name;

  world_ = parent->GetWorld();

  /*
   * Prepare joint state publishing
   */
  if (sdf->HasElement("left_wheel_joint_name"))
  {
    left_wheel_joint_name_ = sdf->GetElement("left_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find left wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("right_wheel_joint_name"))
  {
    right_wheel_joint_name_ = sdf->GetElement("right_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find right wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return;
  }
  joints_[LEFT] = parent->GetJoint(left_wheel_joint_name_);
  joints_[RIGHT] = parent->GetJoint(right_wheel_joint_name_);
  ROS_INFO_STREAM("Left Joint " << joints_[LEFT]);

  if (!joints_[LEFT] || !joints_[RIGHT])
  {
    ROS_ERROR_STREAM("Couldn't find specified wheel joints in the model! [" << node_name_ <<"]");
    return;
  }
  joint_state_.header.frame_id = "Joint States";
  joint_state_.name.push_back(left_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_.name.push_back(right_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  /*
   * Prepare publishing odometry data
   */
  if (sdf->HasElement("publish_tf"))
  {
    publish_tf_ = sdf->GetElement("publish_tf")->Get<bool>();
    if (publish_tf_)
    {
      ROS_INFO_STREAM("Will publish tf." << " [" << node_name_ <<"]");
    }
    else
    {
      ROS_INFO_STREAM("Won't publish tf." << " [" << node_name_ <<"]");
    }
  }
  else
  {
    publish_tf_ = false;
    ROS_INFO_STREAM("Couldn't find the 'publish tf' parameter in the model description."
                     << " Won't publish tf." << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("wheel_separation"))
  {
    wheel_sep_ = sdf->GetElement("wheel_separation")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("wheel_diameter"))
  {
    wheel_diam_ = sdf->GetElement("wheel_diameter")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel diameter parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  if (sdf->HasElement("torque"))
  {
    torque_ = sdf->GetElement("torque")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the torque parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  //odom_reset_sub_ = nh_priv_.subscribe("commands/reset_odometry", 10, &GazeboRosHase::resetOdomCB, this);

  /*
   * Prepare receiving velocity commands
   */
  if (sdf->HasElement("velocity_command_timeout"))
  {
    cmd_vel_timeout_ = sdf->GetElement("velocity_command_timeout")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return;
  }
  last_cmd_vel_time_ = world_->GetSimTime();
  cmd_vel_sub_ = nh_priv_.subscribe("commands/velocity", 100, &GazeboRosHase::cmdVelCB, this);

  prev_update_time_ = world_->GetSimTime();
  ROS_INFO_STREAM("GazeboRosHase plugin ready to go! [" << node_name_ << "]");
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosHase::OnUpdate, this));
}

/*void GazeboRosHase::motorPowerCB(const hase_msgs::MotorPowerPtr &msg)
{
  if ((msg->state == hase_msgs::MotorPower::ON) && (!motors_enabled_))
  {
    motors_enabled_ = true;
    ROS_INFO_STREAM("Motors fired up. [" << node_name_ << "]");
  }
  else if ((msg->state == hase_msgs::MotorPower::OFF) && (motors_enabled_))
  {
    motors_enabled_ = false;
    ROS_INFO_STREAM("Motors taking a rest. [" << node_name_ << "]");
  }
}*/

void GazeboRosHase::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
  last_cmd_vel_time_ = world_->GetSimTime();
  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
}

void GazeboRosHase::resetOdomCB(const std_msgs::EmptyConstPtr &msg)
{
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}

void GazeboRosHase::OnUpdate()
{
  /*
   * First process ROS callbacks
   */
  ros::spinOnce();

  /*
   * Update current time and time step
   */
  common::Time time_now = world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  /*
   * Joint states
   */
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.header.frame_id = "base_link";
  joint_state_.position[LEFT] = joints_[LEFT]->GetAngle(0).Radian();
  joint_state_.velocity[LEFT] = joints_[LEFT]->GetVelocity(0);
  joint_state_.position[RIGHT] = joints_[RIGHT]->GetAngle(0).Radian();
  joint_state_.velocity[RIGHT] = joints_[RIGHT]->GetVelocity(0);
  joint_state_pub_.publish(joint_state_);

  /*
   * Odometry (encoders & IMU)
   */
  odom_.header.stamp = joint_state_.header.stamp;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  // Distance travelled by main wheels
  double d1, d2;
  double dr, da;
  d1 = d2 = 0;
  dr = da = 0;
  d1 = step_time.Double() * (wheel_diam_ / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wheel_diam_ / 2) * joints_[RIGHT]->GetVelocity(0);
  // Can see NaN values here, just zero them out if needed
  if (isnan(d1))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Hase plugin: NaN in d1. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[LEFT]->GetVelocity(0));
    d1 = 0;
  }
  if (isnan(d2))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Hase plugin: NaN in d2. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[RIGHT]->GetVelocity(0));
    d2 = 0;
  }
  dr = (d1 + d2) / 2;
  da = (d2 - d1) / wheel_sep_; // ignored

  // Just as in the Hase driver, the angular velocity is taken directly from the IMU
  vel_angular_ = 0;//imu_->GetAngularVelocity();

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );
  odom_pose_[2] += vel_angular_.z * step_time.Double();
  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = vel_angular_.z;

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setEuler(0,0,odom_pose_[2]);
  odom_.pose.pose.orientation.x = qt.getX();
  odom_.pose.pose.orientation.y = qt.getY();
  odom_.pose.pose.orientation.z = qt.getZ();
  odom_.pose.pose.orientation.w = qt.getW();

  odom_.pose.covariance[0]  = 0.1;
  odom_.pose.covariance[7]  = 0.1;
  odom_.pose.covariance[35] = 0.05;
  odom_.pose.covariance[14] = 1e6;
  odom_.pose.covariance[21] = 1e6;
  odom_.pose.covariance[28] = 1e6;

  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = odom_vel_[2];
  odom_pub_.publish(odom_); // publish odom message

  if (publish_tf_)
  {
    odom_tf_.header = odom_.header;
    odom_tf_.child_frame_id = odom_.child_frame_id;
    odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf_.transform.rotation = odom_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf_);
  }

  /*
   * Propagate velocity commands
   * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
   */
  if (((time_now - last_cmd_vel_time_).Double() > cmd_vel_timeout_))// || !motors_enabled_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }
  joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));
  joints_[LEFT]->SetMaxForce(0, torque_);
  joints_[RIGHT]->SetMaxForce(0, torque_);
}

void GazeboRosHase::spin()
{
  while(ros::ok() && !shutdown_requested_)
  {
    ros::spinOnce();
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosHase);

} // namespace gazebo
