#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

const int PUBLISH_FREQ = 50;

using namespace std;
int button_map[2][7] = {{1, 0, 3, 2, 10, 11, 0}, {1, 0, 4, 3, 6, 7, 8}};

struct TeleopHase
{
    ros::Subscriber joy_sub;
    ros::Publisher pub_vel;

    bool got_first_joy_msg;

    bool toggle_pressed_in_last_msg;

    ros::NodeHandle nh_;
    geometry_msgs::Twist twist;

    enum buttons {X, Y, Z, YAW, L1, R1, SELECT};
    int not_ps3;

    void joyCb(const sensor_msgs::JoyConstPtr joy_msg){

        if (!got_first_joy_msg){
            ROS_INFO("Found joystick with %zu buttons and %zu axes", joy_msg->buttons.size(), joy_msg->axes.size());
            if (joy_msg->buttons.size() != 19 || joy_msg->axes.size() != 28){
                ROS_WARN("This joystick does not look like a PS3-Joystick");
                not_ps3 = 1;
            }
            got_first_joy_msg = true;
        }

        // mapping from joystick to velocity
        float scale = 1;

        twist.linear.x = scale*joy_msg->axes[button_map[not_ps3][X]]; // forward, backward
        twist.angular.z = scale*joy_msg->axes[button_map[not_ps3][YAW]]; // yaw

    }

    TeleopHase(){

        not_ps3 = 0;

        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0;

        got_first_joy_msg = false;

        joy_sub = nh_.subscribe("/joy", 1,&TeleopHase::joyCb, this);

        pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
    }

    void send_cmd_vel(){
        pub_vel.publish(twist);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hase_teleop");

    ROS_INFO("Started Hase joystick-Teleop");

    TeleopHase teleop;
    ros::Rate pub_rate(PUBLISH_FREQ);

    while (teleop.nh_.ok())
    {
        ros::spinOnce();
        teleop.send_cmd_vel();
        pub_rate.sleep();
    }

    return 0;
}
