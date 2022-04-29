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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define foreach BOOST_FOREACH


auto createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    return tf2::toMsg(myQuaternion);
}

bool reset_callback(double *Xk, double *Yk, double *Zk, main_node::Reset::Request &req, main_node::Reset::Response &res)
{
    *Xk = req.X_init;
    *Yk = req.Y_init;
    *Zk = req.Z_init;

    ROS_INFO("posizione resettata");

    return true;
}

void param_callback(int* fmt, main_node::parametersConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d -Level %d", config.fmt, level);
    //*mode = config.mode;
    *fmt= config.fmt;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    double Xk1, Yk1;
    double Zk = 0;
    double Xk = 0;
    double Yk = 0;
    int fmt = 1;

    dynamic_reconfigure::Server<main_node::parametersConfig> dynServer;
    dynamic_reconfigure::Server<main_node::parametersConfig>::CallbackType f;
    f = boost::bind(&param_callback, &fmt, _1, _2);
    dynServer.setCallback(f);

    ros::ServiceServer service = n.advertiseService<main_node::Reset::Request, main_node::Reset::Response>("reset", boost::bind(&reset_callback, &Xk, &Yk, &Zk, _1, _2));

    ros::Publisher velocities = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher positions = n.advertise<nav_msgs::Odometry>("odom", 1000);

    rosbag::Bag bag1;
    bag1.open("/home/scara/robotics/src/main_node/bags/bag1.bag", rosbag::bagmode::Read);

    std::vector <std::string> topics1;
    topics1.push_back(std::string("/wheel_states"));
    rosbag::View view1(bag1, rosbag::TopicQuery(topics1));

    //wheel radius
    double const radius = 0.07;
    //Wheel position along x
    double const l = 0.2;
    //Wheel position along y
    double const w = 0.169;
    double const N = 42;
    double const T = 5;

    double Timek = 0;
    double Timek1 = 0;

    double tick0fr = 0;
    double tick1fr = 0;
    double tick0fl = 0;
    double tick1fl = 0;
    double tick0rr = 0;
    double tick1rr = 0;
    double tick0rl = 0;
    double tick1rl = 0;

    double Vx = 0;
    double Vy = 0;
    double Wz = 0;
    double Vk = 0;

    int i = 0;
    int j = 0;
    int q = 0;
    double min = 0;

    ros::Rate loop_rate(300);

    foreach(rosbag::MessageInstance
    const m, view1)
    {
        i = 0;

        sensor_msgs::JointState::ConstPtr position = m.instantiate<sensor_msgs::JointState>();
        Timek1 = position->header.stamp.nsec * 0.000000001;

        tick1fr = position->position[i + 1];
        tick1fl = position->position[i];
        tick1rr = position->position[i + 3];
        tick1rl = position->position[i + 2];

        if (j == 0) {
            tick0fr = tick1fr;
            tick0fl = tick1fl;
            tick0rr = tick1rr;
            tick0rl = tick1rl;
        }

        double dticksfr = tick1fr - tick0fr;
        double dticksfl = tick1fl - tick0fl;
        double dticksrr = tick1rr - tick0rr;
        double dticksrl = tick1rl - tick0rl;

        double dtime = abs(Timek1 - Timek);

        double Wfl = (dticksfl / dtime) * (1 / N) * (1 / T) * 2 * 3.14;
        double Wfr = (dticksfr / dtime) * (1 / N) * (1 / T) * 2 * 3.14;
        double Wrl = (dticksrl / dtime) * (1 / N) * (1 / T) * 2 * 3.14;
        double Wrr = (dticksrr / dtime) * (1 / N) * (1 / T) * 2 * 3.14;
        //if(Wfl != 0)
        //{
            /*std::cout << "Wfl: " << Wfl << ",";
            std::cout << "Wfr: " << Wfr << ",";
            std::cout << "Wrl: " << Wrl << ",";
            std::cout << "Wrr: " << Wrr << std::endl;
            std::cout << "---------------------- "<< std::endl;*/
        //}


        Vx = (radius / 4) * (Wfl + Wfr + Wrl + Wrr);
        Vy = (radius / 4) * (-Wfl + Wfr + Wrl - Wrr);
        Wz = (radius / ((l + w) * 4)) * (-Wfl + Wfr - Wrl + Wrr);

        /*if(Vx != 0)
        {
            std::cout << "velocità x: " << Vx << std::endl;
            std::cout << "velocità y: " << Vy << std::endl;
            std::cout << "velocità angolare: " << Wz << std::endl;
            std::cout << "-------------" << std::endl;
        }*/

        tick0fr = tick1fr;
        tick0fl = tick1fl;
        tick0rr = tick1rr;
        tick0rl = tick1rl;

        geometry_msgs::Twist velocity;
        velocity.linear.x = Vx;
        velocity.linear.y = Vy;
        velocity.linear.z = Wz;

        Vk = sqrt(Vx * Vx + Vy * Vy);

        j++;

        if(fmt == 0)
        {
            ROS_INFO("EULERO");
            //euler approximation

            Zk = Zk + Wz * abs(Timek1 - Timek);
            Xk1 = Xk + Vx * abs(Timek1 - Timek);
            Yk1 = Yk + Vy * abs(Timek1 - Timek);


            //ROS_INFO("Xk1: %f, Yk1: %f, Zk: %f", Xk1, Yk1, Zk);
        }
        else
        {
            ROS_INFO("RUNGE-KUTTA");
            //runge_kutta
            /*if(Vx < 0 && Vy < 0)
                Vk = -Vk;
            else if(Vx < 0 && Vy > 0)
                Vk = -Vk* cos(3.14);
            else if(Vx > 0 && Vy < 0)
                Vk = Vk* cos(3.14);*/
            Xk1 = Xk + Vk * abs(Timek1 - Timek) * cos(Zk + ((Wz*abs(Timek1 - Timek))/2));
            Yk1 = Yk + Vk * abs(Timek1 - Timek) * sin(Zk + ((Wz*abs(Timek1 - Timek))/2));
            Zk = Zk + Wz * abs(Timek1 - Timek);

            //ROS_INFO("Xk1: %f, Yk1: %f, Zk: %f", Xk1, Yk1, Zk);
        }

        Xk = Xk1;
        Yk = Yk1;
        Timek = Timek1;

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "base_link";
        odometry.pose.pose.position.x = Xk1;
        odometry.pose.pose.position.y = Yk1;
        odometry.pose.pose.position.z = 0;
        odometry.pose.pose.orientation = createQuaternionMsgFromYaw(Zk);

        velocities.publish(velocity);
        positions.publish(odometry);

        ros::spinOnce();

        loop_rate.sleep();
    }

    bag1.close();
    return (0);
}