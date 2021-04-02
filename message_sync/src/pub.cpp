/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-03-10 15:10:13
 * @LastEditors: li
 * @LastEditTime: 2021-03-10 15:19:25
 */
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "message_sync/pointcloud_color.h"
#include "message_sync/pointcloud_color2.h"
#include "std_msgs/UInt8.h"

#include <vector>
#include <iostream>

//#include <unistd.h>
#include <stdio.h>
#include <termios.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_odom_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/yida/yuntai/position",1,true);

    ros::Rate rate(30);

    while (ros::ok())
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;
        pub.publish(odom);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
