#include "mbed.h"
#include <Hase.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_broadcaster.h>
#include <OdometryLite.h>

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
Ticker info;

unsigned char moving = 0; // is the base in motion?

/* The base and odometry frames */
char baseFrame[] = "/base_link";
char odomFrame[] = "/odom";

/* SetPointInfo struct is defined in PIDTypes.h */
Hase::SetPointInfo leftPID, rightPID;

/* OdomInfo struct is defined in PIDTypes.h */
Hase::OdomInfo odomInfo;

/* Create the ROS node handle */
ros::NodeHandle nh;

/* A publisher for OdometryLite data on the /odometry_lite topic. */
hase_firmware::OdometryLite odom_msg;
ros::Publisher odomPub("/hase/odom", &odom_msg);

/* A debugging publisher since nh.loginfo() only takes character constants */
//std_msgs::UInt32 log_msg;
//ros::Publisher logPub("hase/log", &log_msg);

void infoInt(){
    pc.printf("%3.f\t", robot.getRPM(Hase::LEFT_WHEEL));
    pc.printf("%3.f\t", robot.getRPM(Hase::LEFT_WHEEL));
}

void cmdVelCb(const geometry_msgs::Twist& msg){
    float x = msg.linear.x; // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    bt.printf("Linear=%.2f, Angular=%.2f\r\n", x, th);

    /* Reset the auto stop timer */
    //lastMotorCommand = millis();

    if (x == 0 && th == 0) {
        moving = 0;
        robot.setSpeeds(0, 0);
        return;
    }

    /* Indicate that we are moving */
    moving = 1;

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
    /* Set the target speeds in meters per second */
    leftPID.TargetSpeed = spd_left;
    rightPID.TargetSpeed = spd_right;

    /* Convert speeds to encoder ticks per frame */
    leftPID.TargetTicksPerFrame = robot.speedToTicks(leftPID.TargetSpeed);
    rightPID.TargetTicksPerFrame = robot.peedToTicks(rightPID.TargetSpeed);

    robot.setSpeeds(leftPID.TargetTicksPerFrame, rightPID.TargetTicksPerFrame);
}

/* A subscriber for the /cmd_vel topic */
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/hase/cmd_vel", &cmdVelCb);

/* Calculate the odometry update and publish the result */
void updateOdometry() {
  double dt, dleft, dright, dx, dy, dxy_ave, dth, vxy, vth;

  /* Get the time in seconds since the last encoder measurement */
  //dt = nh.now().toSec() - odomInfo.lastOdom.toSec();
  dt = (odomInfo.encoderTime - odomInfo.lastEncoderTime) / 1000.0;

  /* Save the encoder time for the next calculation */
  odomInfo.lastEncoderTime = odomInfo.encoderTime;

  /* Calculate the distance in meters traveled by the two wheels */
  dleft = (leftPID.Encoder - odomInfo.prevLeftEnc) / robot.ticksPerMeter;
  dright = (rightPID.Encoder - odomInfo.prevRightEnc) / robot.ticksPerMeter;

  odomInfo.prevLeftEnc = leftPID.Encoder;
  odomInfo.prevRightEnc = rightPID.Encoder;

  /* Compute the average linear distance over the two wheels */
  dxy_ave = (dleft + dright) / 2.0;

  /* Compute the angle rotated */
  dth = (dright - dleft) / robot.wheelTrack;

  /* Linear velocity */
  vxy = dxy_ave / dt;

  /* Angular velocity */
  vth = dth / dt;

  /* How far did we move forward? */
  if (dxy_ave != 0) {
    dx = cos(dth) * dxy_ave;
    dy = -sin(dth) * dxy_ave;
    /* The total distance traveled so far */
    odomInfo.linearX += (cos(odomInfo.angularZ) * dx - sin(
    odomInfo.angularZ) * dy);
    odomInfo.linearY += (sin(odomInfo.angularZ) * dx + cos(
    odomInfo.angularZ) * dy);
  }

  /* The total angular rotated so far */
  if (dth != 0)
    odomInfo.angularZ += dth;

  /* Represent the rotation as a quaternion */
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(odomInfo.angularZ / 2.0);
  quaternion.w = cos(odomInfo.angularZ / 2.0);

  /* Publish the distances and speeds on the odom topic. Set the timestamp
   > to the last encoder time. */
  odom_msg.header.frame_id = odomFrame;
  odom_msg.child_frame_id = baseFrame;
  odom_msg.header.stamp = odomInfo.encoderStamp;
  odom_msg.pose.position.x = odomInfo.linearX;
  odom_msg.pose.position.y = odomInfo.linearY;
  odom_msg.pose.position.z = 0;
  odom_msg.pose.orientation = quaternion;
  odom_msg.twist.linear.x = vxy;
  odom_msg.twist.linear.y = 0;
  odom_msg.twist.linear.z = 0;
  odom_msg.twist.angular.x = 0;
  odom_msg.twist.angular.y = 0;
  odom_msg.twist.angular.z = vth;

  odomPub.publish(&odom_msg);
}

Ticker updateOdom;

int main() {
    // Init serial
    pc.baud(57600); // Interface with the BBB
    bt.baud(115200); // Interface with Remote Controller
    //nh.initNode();
    //nh.subscribe(cmdVelSub);
    //nh.advertise(odomPub);
    //updateOdom.attach(&updateOdometry, 1.0);

    for (;;)
    {
    robot.setSpeeds(1.0, 1.0);

/*        if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
            robot.setSpeeds(0, 0);
            moving = 0;
        }
*/
        //nh.spinOnce();
        //wait(0.001);
        wait(0.10);
    }
}
