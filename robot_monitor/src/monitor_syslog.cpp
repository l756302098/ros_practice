// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <deque>
#include <string>
#include "vector3.hpp"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <ros/console.h>
#include <thread>
#include <syslog.h>

using namespace std;

nav_msgs::Odometry last_pose;
nav_msgs::Odometry current_pose;
//ros::Time last_time;
//ros::Time current_time;
clock_t last_time, current_time;

yd_vector3::Vector3 currentVe;
yd_vector3::Vector3 lastVe;
double roll, pitch, yaw;
double roll_, pitch_, yaw_;
float threshold_pos;
float yd_distance;
std::string topic;
std::string conrtol_topic;
std::string clear_buff_topic;
int threshold_hz;
int current_hz;
int clear_buff_size;
int clear_count;
int threshold_buffer;
int time_out_num;

//const char *cpu_info;
string cpu_info;

bool isError;
std_msgs::Int32 status;
ros::Publisher control_pub;
ros::Publisher clear_buff_pub;
std::string buffer_topic;

void robotCallback(const nav_msgs::Odometry::ConstPtr &data)
{
  //syslog("I get data ");
  current_pose = *data;
  current_time = clock();

  //double offsetSecs = current_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
  double offsetSecs = (double)(current_time - last_time) / CLOCKS_PER_SEC;
  //std::cout << "offsetSecs:" << offsetSecs << std::endl;
  current_hz = 1 / (offsetSecs);

  currentVe.x = current_pose.pose.pose.position.x;
  currentVe.y = current_pose.pose.pose.position.y;
  currentVe.z = current_pose.pose.pose.position.z;
  yd_distance = yd_vector3::Vector3::TwoPointDistance3D(currentVe, lastVe);
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

  if (yd_distance >= threshold_pos)
  {
    syslog(LOG_INFO, "********************threshold_pos warnning************************");
    syslog(LOG_INFO, "HZ:%d", current_hz);
    syslog(LOG_INFO, "Per CPU Usage:%d", cpu_info);
    syslog(LOG_INFO, "position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "yd_distance offset:%d", yd_distance);
  }
  if (current_hz < threshold_hz)
  {
    syslog(LOG_INFO, "********************hz warnning************************");
    syslog(LOG_INFO, "HZ:%d", current_hz);
    syslog(LOG_INFO, "Per CPU Usage:%d", cpu_info);
    syslog(LOG_INFO, "position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "yd_distance offset:%d", yd_distance);
    syslog(LOG_INFO, "euler angle offset:(%d,%d,%d)", (roll_ - roll), (pitch_ - pitch), (yaw_ - yaw));
  }
  else
  {
    //syslog(LOG_INFO, "current_hz :%d", current_hz);
    // if (isError)
    // {
    //   isError = false;
    //   syslog("restart robot..." << time_out_num);
    //   status.data = 1;
    //   control_pub.publish(status);
    // }
    time_out_num = 0;
  }
  //syslog("time_out_num " << time_out_num);
  // if (time_out_num >= 2 && !isError)
  // {
  //   isError = true;
  //   time_out_num = 0;robot_monitor.launch
  //   syslog("stop robot >< >< ><");
  //   status.data = 0;
  //   control_pub.publish(status);
  // }

  last_pose = current_pose;
  lastVe = currentVe;
  last_time = current_time;
}

void systemCallback(const std_msgs::String::ConstPtr &msg)
{
  cpu_info = msg->data.c_str();
  //std::cout << "cpu_info:" << cpu_info << std::endl;
}

void bufferCallback(const std_msgs::Int16::ConstPtr &msg)
{
  //syslog(LOG_INFO, "buffer size:%d threshold_buffer:%d", msg->data, threshold_buffer);
  if (msg->data >= threshold_buffer)
  {
    if (!isError)
    {
      syslog(LOG_ERR, "stop robot >< >< ><");
      isError = true;
      status.data = 0;
      control_pub.publish(status);
    }
    if (msg->data >= clear_buff_size && isError)
    {
      clear_count++;
      std_msgs::Bool status;
      status.data = true;
      clear_buff_pub.publish(status);
    }
    syslog(LOG_INFO, "********************buffer size warnning************************");
    syslog(LOG_INFO, "HZ:%d", current_hz);
    syslog(LOG_INFO, "Per CPU Usage:%d", cpu_info);
    syslog(LOG_INFO, "position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "yd_distance offset:%d", yd_distance);
    syslog(LOG_INFO, "euler angle offset:(%d,%d,%d)", (roll_ - roll), (pitch_ - pitch), (yaw_ - yaw));
    syslog(LOG_INFO, "clear buffer times: %d", clear_count);
  }
  if (msg->data < threshold_buffer && isError)
  {
    clear_count = 0;
    syslog(LOG_ERR, "restart robot...");
    isError = false;
    status.data = 1;
    control_pub.publish(status);
  }
}

bool heartbeat(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res)
{
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  //init log
  openlog("monitor", LOG_PID, LOG_LOCAL1);

  ros::init(argc, argv, "monitor");
  ros::NodeHandle node;

  ros::param::get("/monitor/threshold_hz", threshold_hz);
  ros::param::get("/monitor/robotpose_topic", topic);
  ros::param::get("/monitor/control_topic", conrtol_topic);
  ros::param::get("/monitor/threshold_pos", threshold_pos);
  ros::param::get("/monitor/buffer_topic", buffer_topic);
  ros::param::get("/monitor/threshold_buffer", threshold_buffer);
  ros::param::get("/monitor/clear_buff_topic", clear_buff_topic);
  ros::param::get("/monitor/clear_buff_size", clear_buff_size);
  clear_count = 0;
  syslog(LOG_INFO, "topic:%s", topic);
  syslog(LOG_INFO, "control_topic:%s", conrtol_topic);

  ros::Subscriber system_sub = node.subscribe<std_msgs::String>("system/info", 1, &systemCallback);
  ros::Subscriber robot_sub = node.subscribe<nav_msgs::Odometry>(topic, 1, &robotCallback);
  ros::Subscriber buffer_sub = node.subscribe<std_msgs::Int16>(buffer_topic, 1, &bufferCallback);
  control_pub = node.advertise<std_msgs::Int32>(conrtol_topic, 1);
  clear_buff_pub = node.advertise<std_msgs::Bool>(clear_buff_topic, 1);

  ros::ServiceServer service = node.advertiseService("/point/heartbeat", heartbeat);

  ros::spin();

  return 0;
}
