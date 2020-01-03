// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <deque>
#include <string>
#include "vector3.hpp"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <ros/console.h>
#include <thread>
#include "log4z.h"

using namespace std;
using namespace zsummer::log4z;

nav_msgs::Odometry last_pose;
nav_msgs::Odometry current_pose;
ros::Time last_time;
ros::Time current_time;
yd_vector3::Vector3 currentVe;
yd_vector3::Vector3 lastVe;
double roll, pitch, yaw;
double roll_, pitch_, yaw_;
int current_hz;
std::string topic;
std::string conrtol_topic;
int threshold_hz;
//const char *cpu_info;
string cpu_info;
int time_out_num;
bool isError;
std_msgs::Int16 status;
ros::Publisher control_pub;

void robotCallback(const nav_msgs::Odometry::ConstPtr &data)
{
  //ROS_INFO_STREAM("I get data ");
  current_pose = *data;
  current_time = ros::Time::now();

  double offsetSecs = current_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
  current_hz = 1 / (offsetSecs);

  if (current_hz < threshold_hz)
  {
    ROS_INFO_STREAM("********************"
                    << "profiler"
                    << "************************");
    ROS_INFO_STREAM("HZ:" << current_hz);
    ROS_INFO_STREAM("Per CPU Usage:" << cpu_info);
    ROS_INFO_STREAM("position :(" << current_pose.pose.pose.position.x << "," << current_pose.pose.pose.position.y << "," << current_pose.pose.pose.position.z << ")");
    currentVe.x = current_pose.pose.pose.position.x;
    currentVe.y = current_pose.pose.pose.position.y;
    currentVe.z = current_pose.pose.pose.position.z;
    double distance = yd_vector3::Vector3::TwoPointDistance3D(currentVe, lastVe);
    ROS_INFO_STREAM("distance offset:" << distance);
    //rotation
    geometry_msgs::Quaternion msg = current_pose.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //last
    geometry_msgs::Quaternion msg_ = last_pose.pose.pose.orientation;
    tf::Quaternion quat_;
    tf::quaternionMsgToTF(msg_, quat_);
    tf::Matrix3x3(quat_).getRPY(roll_, pitch_, yaw_);

    ROS_INFO_STREAM("euler angle offset:(" << (roll_ - roll) << "," << (pitch_ - pitch) << "," << (yaw_ - yaw) << ")");
    ROS_INFO_STREAM("----------------------------------------------------");
    time_out_num++;
  }
  else
  {
    ROS_INFO_STREAM("current_hz :" << current_hz);
    if (isError)
    {
      isError = false;
      ROS_INFO_STREAM("restart robot..." << time_out_num);
      status.data = 1;
      control_pub.publish(status);
    }
    time_out_num = 0;
  }
  ROS_INFO_STREAM("time_out_num " << time_out_num);
  if (time_out_num >= 2)
  {
    isError = true;
    time_out_num = 0;
    ROS_INFO_STREAM("stop robot >< >< ><");
    status.data = 0;
    control_pub.publish(status);
  }

  last_pose = current_pose;
  lastVe = currentVe;
  last_time = current_time;
}

void systemCallback(const std_msgs::String::ConstPtr &msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  cpu_info = msg->data.c_str();
}

int main(int argc, char **argv)
{
  //init log
  ILog4zManager::getRef().start();

  ros::init(argc, argv, "monitor");
  ros::NodeHandle node;

  ros::param::get("/monitor/threshold_hz", threshold_hz);
  ros::param::get("/monitor/robotpose_topic", topic);
  ros::param::get("/monitor/conrtol_topic", conrtol_topic);

  ROS_INFO_STREAM("topic:" << topic);

  ros::Subscriber system_sub = node.subscribe("system/info", 1, &systemCallback);
  ros::Subscriber robot_sub = node.subscribe<nav_msgs::Odometry>(topic, 1, &robotCallback);
  control_pub = node.advertise<std_msgs::Int16>(conrtol_topic, 10);

  ros::spin();

  return 0;
}
