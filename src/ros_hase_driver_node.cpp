#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "hase/Hase.h"

double wheel_diameter = 0.0685;
double wheel_track = 0.2152;
double gear_reduction = 1;
double encoder_resolution = 5;
double ticks_per_meter;
double speed_koef = 10;

hase::Hase robot;

void cmd_vel_received(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	double epsilon = std::numeric_limits<double>::epsilon();
	double linear_speed = cmd_vel->linear.x; // m/s
	double angular_speed = cmd_vel->angular.z; // rad/s

	ROS_INFO("Velocity received: %.2f %.2f", linear_speed, angular_speed);

	if( linear_speed <  epsilon && linear_speed > -epsilon ) {
		// zero linear speed - turn in place
		double speed = angular_speed;
		robot.drive_2wd(-speed, speed);
	}else if( angular_speed < epsilon && angular_speed > -epsilon ) {
		// zero angular speed - pure forward/backward motion
		double speed = linear_speed;
		robot.drive_2wd(speed, speed);
	}else {
		// Rotation about a point in space
		//$TODO
		double left = linear_speed - angular_speed;
		double right = linear_speed + angular_speed;
		//hase.move_robot_2WD(speed, -speed);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hase_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmd_vel_received);

	while (ros::ok()) {
		ros::spin();
	}

	ROS_INFO("End");
	return 0;
}
