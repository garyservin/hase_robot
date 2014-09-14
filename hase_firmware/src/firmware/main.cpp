#include "mbed.h"
#include <Hase.h>
#include <ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <ImuLite.h>

// Rate at which gyroscope is sampled
#define IMU_RATE       10     // Hz
const float IMU_INTERVAL = 1.0 / IMU_RATE;

// Rate at which velocities are sampled
#define TWIST_RATE       10     // Hz
const float TWIST_INTERVAL = 1.0 / TWIST_RATE;

// LEDs
DigitalOut led2 ( LED2 );
DigitalOut led3 ( LED3 );
DigitalOut led4 ( LED4 );

Hase robot;
Ticker twist;
Ticker imu;

// Callback method for cmd_vel
void cmdVelCb(const geometry_msgs::Twist& msg);

// The base frame
char baseFrame[] = "/base_link";

// Create the ROS node handle
ros::NodeHandle nh;

// A publisher for TwistStamped data on the /hase/twist topic.
geometry_msgs::TwistStamped twist_msg;
ros::Publisher pub_twist("/hase/twist_lite", &twist_msg);

// ImuLite publisher
hase_firmware::ImuLite imu_msg;
ros::Publisher pub_imu("/hase/imu_lite", &imu_msg);

// A subscriber for the /cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/hase/cmd_vel", &cmdVelCb);

void cmdVelCb(const geometry_msgs::Twist& msg){
    float x = msg.linear.x; // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    if (x == 0.0 && th == 0.0) {
        spd_left = spd_right = 0.0;
    }
    else if (x == 0.0) {
        // Turn in place
        spd_right = th * robot.wheelTrack / 2.0;
        spd_left = -spd_right;
    }
    else if (th == 0.0){
        // Pure forward/backward motion
        spd_left = spd_right = x;
    }
    else {
        // Rotation about a point in space
        spd_left = x - th * robot.wheelTrack / 2.0;
        spd_right = x + th * robot.wheelTrack / 2.0;
    }

    robot.info("Received cmd_vel: x=%.2f, th=%.2f", x, th);
    robot.info("Wheel Speeds to: L=%.2f, R=%.2f", spd_left, spd_right);

    robot.setSpeeds(spd_left, spd_right);
}

// Calculate the twist and publish the result
void sendTwist() {

    double vleft, vright, vxy, vth;

    vleft = robot.getWheelSpeed(Hase::LEFT_WHEEL);
    vright = robot.getWheelSpeed(Hase::RIGHT_WHEEL);

    // Linear velocity
    vxy = (vright + vleft) / 2.0;

    // Angular velocity
    vth = (vright - vleft) / robot.wheelTrack;

    // Publish the speeds on the twist topic. Set the timestamp to the last encoder time.
    twist_msg.header.frame_id = baseFrame;
    twist_msg.header.stamp = nh.now();
    twist_msg.twist.linear.x = vxy;
    twist_msg.twist.linear.y = 0;
    twist_msg.twist.linear.z = 0;
    twist_msg.twist.angular.x = 0;
    twist_msg.twist.angular.y = 0;
    twist_msg.twist.angular.z = vth;

    pub_twist.publish(&twist_msg);
}

void sendImu(){
    imu_msg.header.frame_id =  baseFrame;
    imu_msg.header.stamp = nh.now();

    imu_msg.angular_velocity.z = robot.getYawSpeed();

    pub_imu.publish(&imu_msg);
}

int main() {
    nh.initNode();
    nh.subscribe(cmdVelSub);

    nh.advertise(pub_twist);
    twist.attach(&sendTwist, TWIST_INTERVAL);

    nh.advertise(pub_imu);
    imu.attach(&sendImu, IMU_INTERVAL);

    robot.setSpeeds(0.0, 0.0);
    while (1)
    {
        nh.spinOnce();
        wait(0.01);
    }
}
