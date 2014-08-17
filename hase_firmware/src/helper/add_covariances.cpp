#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <hase_firmware/ImuLite.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

ros::Publisher twist_pub;
ros::Publisher imu_pub;

void twist_cb( const geometry_msgs::TwistStamped::ConstPtr & msg ) {
    // Twist message
    geometry_msgs::TwistWithCovarianceStamped out;

    out.header = msg->header;
    out.twist.twist = msg->twist;

    // TODO: check correct covariances
    out.twist.covariance[0] = 0.3;
    out.twist.covariance[7] = 0.0;
    out.twist.covariance[14] = 0.0;
    out.twist.covariance[21] = 0.0;
    out.twist.covariance[28] = 0.0;
    out.twist.covariance[35] = 0.3;

    twist_pub.publish(out);
}

void imu_cb( const hase_firmware::ImuLite::ConstPtr & msg ) {
    // IMU message
    sensor_msgs::Imu imu;

    imu.header = msg->header;
    imu.angular_velocity = msg->angular_velocity;

    for (int i = 0; i < 9; i++)
    {
        imu.linear_acceleration_covariance[i] = 0.0;
        imu.angular_velocity_covariance[i] = 0.0;
        imu.orientation_covariance[i] = 0.0;
    }

    imu_pub.publish(imu);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "add_covariances");

    ros::NodeHandle nh;

    ros::Subscriber twist_sub = nh.subscribe("/hase/twist_lite", 2, &twist_cb);
    ros::Subscriber imu_sub = nh.subscribe("/hase/imu_lite", 2, &imu_cb);

    twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/hase/twist", 2);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/hase/imu", 2);

    ros::spin();
}
