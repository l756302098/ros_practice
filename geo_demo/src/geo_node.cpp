/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-05-11 16:38:53
 * @LastEditors: li
 * @LastEditTime: 2021-05-11 17:51:50
 */
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
using namespace std;
using namespace GeographicLib;
Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
LocalCartesian geoConverter;
ros::Publisher state_pub_;
nav_msgs::Path ros_path_;
bool init;


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    //初始化
    if(!init)
    {
        init = true;
        geoConverter.Reset(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
    } 
    else
    {
        //计算相对位置
        double x, y, z;
        geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude,x,y,z);
        cout << "result " <<x << " " << y << " " << z << "\n";
        //发布轨迹
        ros_path_.header.frame_id = "path";
        ros_path_.header.stamp = ros::Time::now();  

        geometry_msgs::PoseStamped pose;
        pose.header = ros_path_.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        ros_path_.poses.push_back(pose);
        ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );
        state_pub_.publish(ros_path_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "geo_demo_node");
    ros::NodeHandle nh("~");
    ros::Subscriber pose_sub=nh.subscribe("/gps/fix",10,gpsCallback);
    state_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);
    ros::Rate rate(30);
    while (ros::ok())
    {
        //std::cout << "update" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
