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
  //LOGI("I get data ");
  current_pose = *data;
  current_time = ros::Time::now();

  double offsetSecs = current_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
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
    LOGW("********************"
         << "threshold_pos warnning"
         << "************************");
    LOGW("HZ:" << current_hz);
    LOGW("Per CPU Usage:" << cpu_info);
    LOGW("position :(" << current_pose.pose.pose.position.x << "," << current_pose.pose.pose.position.y << "," << current_pose.pose.pose.position.z << ")");
    LOGW("yd_distance offset:" << yd_distance);
  }
  if (current_hz < threshold_hz)
  {
    LOGW("********************"
         << "hz warnning"
         << "************************");
    LOGW("HZ:" << current_hz);
    LOGW("Per CPU Usage:" << cpu_info);
    LOGW("position :(" << current_pose.pose.pose.position.x << "," << current_pose.pose.pose.position.y << "," << current_pose.pose.pose.position.z << ")");
    LOGW("yd_distance offset:" << yd_distance);
    LOGW("euler angle offset:(" << (roll_ - roll) << "," << (pitch_ - pitch) << "," << (yaw_ - yaw) << ")");
    //time_out_num++;
  }
  else
  {
    LOGI("current_hz :" << current_hz);
    // if (isError)
    // {
    //   isError = false;
    //   LOGI("restart robot..." << time_out_num);
    //   status.data = 1;
    //   control_pub.publish(status);
    // }
    time_out_num = 0;
  }
  //LOGI("time_out_num " << time_out_num);
  // if (time_out_num >= 2 && !isError)
  // {
  //   isError = true;
  //   time_out_num = 0;robot_monitor.launch
  //   LOGI("stop robot >< >< ><");
  //   status.data = 0;
  //   control_pub.publish(status);
  // }

  last_pose = current_pose;
  lastVe = currentVe;
  last_time = current_time;
}

void systemCallback(const std_msgs::String::ConstPtr &msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  cpu_info = msg->data.c_str();
}

void bufferCallback(const std_msgs::Int16::ConstPtr &msg)
{
  LOGI("buffer size" << msg->data << " threshold_buffer:" << threshold_buffer);
  if (msg->data >= threshold_buffer)
  {
    if (!isError)
    {
      LOGE("stop robot >< >< ><");
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
    LOGW("********************"
         << "buffer size warnning"
         << "************************");
    LOGW("HZ:" << current_hz);
    LOGW("Per CPU Usage:" << cpu_info);
    LOGW("position :(" << current_pose.pose.pose.position.x << "," << current_pose.pose.pose.position.y << "," << current_pose.pose.pose.position.z << ")");
    LOGW("yd_distance offset:" << yd_distance);
    LOGW("euler angle offset:(" << (roll_ - roll) << "," << (pitch_ - pitch) << "," << (yaw_ - yaw) << ")");
    LOGW("clear buffer times: " << clear_count);
  }
  if (msg->data < threshold_buffer && isError)
  {
    clear_count = 0;
    LOGE("restart robot...");
    isError = false;
    status.data = 1;
    control_pub.publish(status);
  }
}

bool heartbeat(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res)
{
  cout << "alive ..." << endl;
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  //init log
  ILog4zManager::getRef().start();

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
  LOGI("topic:" << topic);
  LOGI("control_topic:" << conrtol_topic);

  ros::Subscriber system_sub = node.subscribe<std_msgs::String>("system/info", 1, &systemCallback);
  ros::Subscriber robot_sub = node.subscribe<nav_msgs::Odometry>(topic, 1, &robotCallback);
  ros::Subscriber buffer_sub = node.subscribe<std_msgs::Int16>(buffer_topic, 1, &bufferCallback);
  control_pub = node.advertise<std_msgs::Int32>(conrtol_topic, 1);
  clear_buff_pub = node.advertise<std_msgs::Bool>(clear_buff_topic, 1);

  ros::ServiceServer service = node.advertiseService("/point/heartbeat", heartbeat);

  ros::spin();

  return 0;
}
