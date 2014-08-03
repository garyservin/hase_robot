#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

ros::Publisher twist_pub;

void twist_cb( const geometry_msgs::TwistStamped::ConstPtr & msg ) {
    // Twist message
    geometry_msgs::TwistWithCovarianceStamped out;
    out.header = msg->header;
    out.twist.twist = msg->twist;

    out.twist.covariance[0] = 0.3;
    out.twist.covariance[7] = 0.0;
    out.twist.covariance[14] = 0.0;
    out.twist.covariance[21] = 0.0;
    out.twist.covariance[28] = 0.0;
    out.twist.covariance[35] = 0.3;

    twist_pub.publish(out);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "add_covariance_twist");

    ros::NodeHandle nh;

    ros::Subscriber s = nh.subscribe("/hase/twist_lite", 2, &twist_cb);

    twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/hase/twist", 2);

    ros::spin();
}
