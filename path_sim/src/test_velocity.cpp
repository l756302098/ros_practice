#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

ros::Time start_time, t;
ros::Duration duration;

using namespace std;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "test");
//     ros::NodeHandle nh;

//     ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1, true);

//     start_time = ros::Time::now();
//     duration = ros::Duration(10);
//     geometry_msgs::Twist motor_control;
//     motor_control.linear.x = 0;
//     motor_control.angular.z = 0.314;
//     cmd_pub.publish(motor_control);

//     ros::Rate loop_rate(10);

//     while (ros::ok())
//     {
//         t = ros::Time::now();
//         if (t - start_time > duration)
//         {
//             cout << "game over!" << endl;
//             motor_control.linear.x = 0;
//             motor_control.angular.z = 0;
//             cmd_pub.publish(motor_control);
//             //ros::shutdown();
//         }
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_velocity");

    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1, true);

    start_time = ros::Time::now();
    duration = ros::Duration(10);
    geometry_msgs::Twist motor_control;

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        cout << count << endl;
        t = ros::Time::now();
        if (t - start_time > duration)
        {
            cout << "game over!" << endl;
            motor_control.linear.x = 0;
            motor_control.angular.z = 0;
            cmd_pub.publish(motor_control);
        }
        else
        {
            motor_control.linear.x = 0.2;
            motor_control.angular.z = 0;
            cmd_pub.publish(motor_control);
        }
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
