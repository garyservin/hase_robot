#include "mbed.h"
#include <Hase.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/TwistStamped.h>

#define BT_TX   p28
#define BT_RX   p27

//#define DEBUG_SERIAL

// LEDs
DigitalOut led1 ( LED1 );
DigitalOut led2 ( LED2 );
DigitalOut led3 ( LED3 );
DigitalOut led4 ( LED4 );

// Serial Communications
Serial pc ( USBTX, USBRX ); // Serial Comm to PC
Serial bt ( BT_TX, BT_RX ); // Serial Comm to PC/Control via Bluetooth

Hase robot;
Ticker led;
Ticker twist;

// The base frame
char baseFrame[] = "/base_link";

// Create the ROS node handle
ros::NodeHandle nh;

// A publisher for TwistStamped data on the /hase/twist topic.
geometry_msgs::TwistStamped twist_msg;
ros::Publisher twistPub("/hase/twist_lite", &twist_msg);

void ledInt(){
    led1 = !led1;
}

void cmdVelCb(const geometry_msgs::Twist& msg){
    float x = msg.linear.x; // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    bt.printf("Linear=%.2f, Angular=%.2f\r\n", x, th);

    // Reset the auto stop timer
    //lastMotorCommand = millis();

    if (x == 0 && th == 0) {
        //moving = 0;
        robot.setSpeeds(0, 0);
        return;
    }

    // Indicate that we are moving
    //moving = 1;

    if (x == 0) {
        // Turn in place
        spd_right = th * robot.wheelTrack / 2.0;
        spd_left = -spd_right;
    }
    else if (th == 0) {
        // Pure forward/backward motion
        spd_left = spd_right = x;
    }
    else {
        // Rotation about a point in space
        spd_left = x - th * robot.wheelTrack / 2.0;
        spd_right = x + th * robot.wheelTrack / 2.0;
    }

    bt.printf("Left=%.2f, Right=%.2f\r\n", spd_left, spd_right);
    // Set the target speeds in meters per second
    //leftPID.TargetSpeed = spd_left;
    //rightPID.TargetSpeed = spd_right;

    // Convert speeds to encoder ticks per frame
    //leftPID.TargetTicksPerFrame = robot.speedToTicks(leftPID.TargetSpeed);
    //rightPID.TargetTicksPerFrame = robot.speedToTicks(rightPID.TargetSpeed);

    //robot.setSpeeds(leftPID.TargetTicksPerFrame, rightPID.TargetTicksPerFrame);
}

// A subscriber for the /cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);

// Calculate the twisr update and publish the result
void updateTwist() {
  //bt.printf("%d\r", robot.getRPM(Hase::LEFT_WHEEL));

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

  twistPub.publish(&twist_msg);
}

int main() {
    // Init serial
    pc.baud(57600); // Interface with the BBB
    bt.baud(115200); // Interface with Remote Controller
    nh.initNode();
    nh.subscribe(cmdVelSub);
    nh.advertise(twistPub);
    led.attach(&ledInt, 0.5);
    twist.attach(&updateTwist, 0.1);

    while (1)
    {
        //robot.setSpeeds(1.0, 1.0);

        /*
        if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
            robot.setSpeeds(0, 0);
            moving = 0;
        }
        */

        nh.spinOnce();
        wait(0.001);
    }
}
