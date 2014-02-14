#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "hase/MoveRobot.h"

double wheel_diameter = 0.0685;
double wheel_track = 0.2152;
double gear_reduction = 1;
double encoder_resolution = 5;
double ticks_per_meter;
double speed_koef = 10;

void cmd_vel_received(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    double linear_speed = cmd_vel->linear.x; // m/s
    double angular_speed = cmd_vel->angular.z; // rad/s
    ROS_INFO("Velocity received: %.2f %.2f", linear_speed, angular_speed);

    if( linear_speed <  std::numeric_limits<double>::epsilon() &&
            linear_speed > -std::numeric_limits<double>::epsilon() ) {
        // zero linear speed - turn in place
        double speed = angular_speed;
        //move_robot_2WD(speed, -speed);
    }
    else if( angular_speed <  std::numeric_limits<double>::epsilon() &&
             angular_speed > -std::numeric_limits<double>::epsilon() ) {
        // zero angular speed - pure forward/backward motion
        double speed = linear_speed;
        //move_robot_2WD(speed, speed);
    }
    else {
        // Rotation about a point in space
        //$TODO
        double left = linear_speed - angular_speed;
        double right = linear_speed + angular_speed;

        //move_robot_2WD(speed, -speed);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hase_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmd_vel_received);

    init_robot();

    while (ros::ok()) {
        ros::spin();
    }

    close_robot();

    ROS_INFO("End");
    return 0;
}
