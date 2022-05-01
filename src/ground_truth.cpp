#include <rosbag/bag.h>
#include "ros/ros.h"
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include "main_node/wheels.h"
#include "main_node/Reset.h"
#include <dynamic_reconfigure/server.h>
#include "main_node/parametersConfig.h"
#define foreach BOOST_FOREACH

int main(int argc, char **argv) {
    ros::init(argc, argv, "GT");
    ros::NodeHandle n;

    rosbag::Bag bag;
    bag.open("/home/scara/robotics/src/main_node/bags/bag2.bag", rosbag::bagmode::Read);

    std::vector <std::string> topics;
    topics.push_back(std::string("/robot/pose"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Publisher GT_pos = n.advertise<nav_msgs::Odometry>("GT_pos", 1000);

    ros::Rate loop_rate(1000);

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseStamped::ConstPtr position = m.instantiate<geometry_msgs::PoseStamped>();

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "base_link";
        odometry.pose.pose.position.x = position->pose.position.x;
        odometry.pose.pose.position.y = position->pose.position.y;
        odometry.pose.pose.position.z = position->pose.position.y;
        odometry.pose.pose.orientation.x = position->pose.orientation.x;
        odometry.pose.pose.orientation.y = position->pose.orientation.y;
        odometry.pose.pose.orientation.z = position->pose.orientation.z;
        odometry.pose.pose.orientation.w = position->pose.orientation.w;

        GT_pos.publish(odometry);

        ros::spinOnce();

        loop_rate.sleep();
    }
    bag.close();
    return (0);
}