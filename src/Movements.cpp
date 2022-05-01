#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "main_node/wheels.h"

//wheel radius
double const radius = 0.07;
//Wheel position along x
double const l = 0.2;
//Wheel position along y
double const w = 0.169;
double const N = 42;
double const T = 5;

ros::Publisher wheel_vel;

void printVelocities(const geometry_msgs::Twist velocity) {

    main_node::wheels wheel_speed_msg;

    wheel_speed_msg.Wfl = (1/radius)*(velocity.linear.x-velocity.linear.y-(l+w)*velocity.linear.z);
    wheel_speed_msg.Wfr = (1/radius)*(velocity.linear.x+velocity.linear.y+(l+w)*velocity.linear.z);
    wheel_speed_msg.Wrl = (1/radius)*(velocity.linear.x+velocity.linear.y-(l+w)*velocity.linear.z);
    wheel_speed_msg.Wrr = (1/radius)*(velocity.linear.x-velocity.linear.y+(l+w)*velocity.linear.z);

    ROS_INFO("Wfl: %f, Wfr: %f, Wrl: %f, Wrr: %f", wheel_speed_msg.Wfl, wheel_speed_msg.Wfr, wheel_speed_msg.Wrl, wheel_speed_msg.Wrr);
    std::cout << "---------------------- "<< std::endl;

    wheel_vel.publish(wheel_speed_msg);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    wheel_vel = n.advertise<main_node::wheels>("wheels_rpm", 1000);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, printVelocities);

    ros::spin();

    return 0;
}
