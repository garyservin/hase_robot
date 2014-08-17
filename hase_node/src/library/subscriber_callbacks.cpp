#include <hase_node/hase_ros.hpp>

namespace hase
{

void HaseRos::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
        // For now assuming this is in the robot frame, but probably this
        // should be global frame and require a transform
        //double vx = msg->linear.x;        // in (m/s)
        //double wz = msg->angular.z;       // in (rad/s)
        ROS_DEBUG_STREAM("Hase : velocity command received [" << msg->linear.x << "],[" << msg->angular.z << "]");
        //hase.setBaseControl(msg->linear.x, msg->angular.z);

        double epsilon = std::numeric_limits<double>::epsilon();
        double linear_speed = msg->linear.x; // m/s
        double angular_speed = msg->angular.z; // rad/s
        ROS_DEBUG("Velocity received: %.2f %.2f", linear_speed, angular_speed);

    if(usebbb_){
        if( linear_speed <  epsilon && linear_speed > -epsilon ) {
            // zero linear speed - turn in place
            double speed = angular_speed;
            #ifdef USE_BBB
            hase.drive_2wd(-speed, speed);
            #endif
        }else if( angular_speed < epsilon && angular_speed > -epsilon ) {
            // zero angular speed - pure forward/backward motion
            double speed = linear_speed;
            #ifdef USE_BBB
            hase.drive_2wd(speed, speed);
            #endif
        }else {
            // Rotation about a point in space
            //$TODO
            double left = linear_speed - angular_speed;
            double right = linear_speed + angular_speed;
            //hase.move_robot_2WD(speed, -speed);
        }
    }else{
        geometry_msgs::Twist twist = *msg;
        twist_mbed_publisher.publish(twist);
    }

  return;
}

} // namespace hase
