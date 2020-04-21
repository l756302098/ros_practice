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
#include <sstream>
#include <cstring>
#include <velodyne_msgs/VelodyneScan.h>

using namespace std;

class monitor_ros_node
{
private:
  ros::NodeHandle node;
  ros::Subscriber robot_sub;
  ros::Subscriber buffer_sub;
  ros::Subscriber jump_sub;
  ros::Subscriber velodyne_sub;
  nav_msgs::Odometry last_pose;
  nav_msgs::Odometry current_pose;

  velodyne_msgs::VelodyneScan current_scan;
  velodyne_msgs::VelodyneScan last_scan;

  //ros::Time last_time;
  ros::Time current_time;
  clock_t last_time;
  yd_vector3::Vector3 currentVe;
  yd_vector3::Vector3 lastVe;
  double roll, pitch, yaw;
  double roll_, pitch_, yaw_;
  float threshold_pos;
  float yd_distance;
  std::string topic;
  std::string conrtol_topic;
  std::string clear_buff_topic;
  std::string stop_hdl_topic;
  std::string jump_topic;
  //hz 阈值
  int threshold_hz;
  int current_hz;
  // 清除buffer size 的大小
  int clear_buff_size;
  int clear_count;
  //buffer 阈值,超过停止运动
  int threshold_buffer;
  //连续 clear buffer 的最大值
  int threshold_clear_count;
  //连续 clear buffer 的次数
  int clear_buffer_total;
  int threshold_jump_time;
  //连续多久恢复行走
  int laster_recover_count;
  int laster_manual_count;
  //连续多次clear buffer之后，让hdl 休息一段时间
  int laster_sleep_time;
  //jump_count
  int jump_total;
  int velodyne_threshold_hz;

  //const char *cpu_info;
  string cpu_info;
  bool hdl_is_sleep;
  bool laser_is_delay;
  ros::Time sleep_time;
  //std_msgs::Int32 status;
  ros::Publisher control_pub;
  ros::Publisher clear_buff_pub;
  //ros::Publisher stop_hdl_pub;
  std::string buffer_topic;

public:
  bool isError;
  bool robot_is_stop;
  monitor_ros_node();
  ~monitor_ros_node();

  void robotCallback(const nav_msgs::Odometry::ConstPtr &data);
  void jumpCallback(const std_msgs::Bool::ConstPtr &msg);
  void bufferCallback(const std_msgs::Int16::ConstPtr &msg);
  void update();
  void set_hdl_status(bool isOpen);
};
monitor_ros_node::monitor_ros_node()
{
  ros::param::get("/monitor/threshold_hz", threshold_hz);
  ros::param::get("/monitor/robotpose_topic", topic);
  ros::param::get("/monitor/control_topic", conrtol_topic);
  ros::param::get("/monitor/threshold_pos", threshold_pos);
  ros::param::get("/monitor/buffer_topic", buffer_topic);
  ros::param::get("/monitor/threshold_buffer", threshold_buffer);
  ros::param::get("/monitor/stop_hdl_topic", stop_hdl_topic);
  ros::param::get("/monitor/jump_topic", jump_topic);
  ros::param::get("/monitor/threshold_jump_time", threshold_jump_time);
  ros::param::get("/monitor/threshold_clear_count", threshold_clear_count);
  ros::param::get("/monitor/laster_recover_count", laster_recover_count);
  ros::param::get("/monitor/laster_sleep_time", laster_sleep_time);
  ros::param::get("/monitor/clear_buff_size", clear_buff_size);
  ros::param::get("/monitor/clear_buff_topic", clear_buff_topic);
  //velodyne_threshold_hz
  ros::param::get("/monitor/velodyne_threshold_hz", velodyne_threshold_hz);
  clear_count = 0;
  isError = false;
  hdl_is_sleep = false;
  laser_is_delay = false;
  robot_is_stop = false;
  std::cout
      << "buffer_topic:" << buffer_topic << std::endl;
  std::cout << "stop_hdl_topic:" << stop_hdl_topic << std::endl;
  std::cout << "jump_topic:" << jump_topic << std::endl;
  std::cout << "laster_recover_count:" << laster_recover_count << std::endl;
  std::cout << "laster_sleep_time:" << laster_sleep_time << std::endl;
  std::cout << "control topic " << conrtol_topic << std::endl;
  std::cout << "clear_buff_size " << clear_buff_size << std::endl;
  std::cout << "clear_buff_topic " << clear_buff_topic << std::endl;
  std::cout << "velodyne_threshold_hz " << velodyne_threshold_hz << std::endl;
  //velodyne_sub = node.subscribe("velodyne_packets", 10, &monitor_ros_node::processScan, this);
  robot_sub = node.subscribe<nav_msgs::Odometry>(topic, 1, &monitor_ros_node::robotCallback, this);
  buffer_sub = node.subscribe<std_msgs::Int16>(buffer_topic, 1, &monitor_ros_node::bufferCallback, this);
  jump_sub = node.subscribe<std_msgs::Bool>(jump_topic, 1, &monitor_ros_node::jumpCallback, this);
  control_pub = node.advertise<std_msgs::Int32>(conrtol_topic, 1);
  clear_buff_pub = node.advertise<std_msgs::Bool>(clear_buff_topic, 1);
  //stop_hdl_pub = node.advertise<std_msgs::Bool>(stop_hdl_topic, 1);
  //ros::ServiceServer service = node.advertiseService("/point/heartbeat", &heartbeat);
}

monitor_ros_node::~monitor_ros_node()
{
  ostringstream oss;
  oss << "~monitor_ros_node";
  syslog(LOG_INFO, oss.str().c_str());
}

void monitor_ros_node::set_hdl_status(bool isStop){
    ros::param::set("/stop_receive_laser", isStop);
}

void monitor_ros_node::update()
{
  //std::cout << "monitor_ros_node update " << std::endl;
  if (hdl_is_sleep)
  {
    current_time = ros::Time::now();
    int sleeping_time = current_time.toSec() - sleep_time.toSec();
    //std::cout << "sleeping_time " << sleeping_time << std::endl;
    ostringstream oss;
    oss << "sleeping_time:" << sleeping_time;
    syslog(LOG_INFO, oss.str().c_str());
    if (sleeping_time > laster_sleep_time)
    {
      //syslog(LOG_ERR, "restart hdl >< >< ><");
      oss.str("");
      oss << "restart hdl >< >< ><";
      syslog(LOG_INFO, oss.str().c_str());
      //恢复hdl
      // std_msgs::Bool status;
      // status.data = false;
      // stop_hdl_pub.publish(status);
      set_hdl_status(false);
      hdl_is_sleep = false;
      clear_count = 0;
    }
  }
}

void monitor_ros_node::robotCallback(const nav_msgs::Odometry::ConstPtr &data)
{
  current_pose = *data;
  clock_t start_time = clock();

  //double offsetSecs = current_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
  double offsetSecs = (double)(start_time - last_time) / CLOCKS_PER_SEC;
  //double offsetSecs = current_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
  current_hz = 1 / (offsetSecs);

  currentVe.x = current_pose.pose.pose.position.x;
  currentVe.y = current_pose.pose.pose.position.y;
  currentVe.z = current_pose.pose.pose.position.z;
  yd_distance = yd_vector3::Vector3::TwoPointDistance3D(currentVe, lastVe);
  //std::cout << "yd_distance:" << yd_distance << std::endl;
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
    //syslog(LOG_INFO, "HZ:%d", current_hz);
    syslog(LOG_INFO, "position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "yd_distance offset:%d", yd_distance);
  }

  last_pose = current_pose;
  lastVe = currentVe;
  last_time = start_time;
}

void monitor_ros_node::jumpCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data)
  {
    jump_total++;
    syslog(LOG_ERR, "jump callback count:%s", jump_total);
    //syslog(LOG_INFO, "jump HZ:%d", current_hz);
    syslog(LOG_INFO, "jump position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "jump yd_distance offset:%d", yd_distance);
    syslog(LOG_INFO, "jump euler angle offset:(%d,%d,%d)", (roll_ - roll), (pitch_ - pitch), (yaw_ - yaw));
    syslog(LOG_INFO, "jump clear buffer times: %d", clear_count);
  }
}

void monitor_ros_node::bufferCallback(const std_msgs::Int16::ConstPtr &msg)
{
  if (msg->data >= threshold_buffer)
  {
    ostringstream ss;
    ss << "buffer size:" << msg->data << " threshold_buffer:" << threshold_buffer;
    syslog(LOG_INFO, ss.str().c_str());
    laster_manual_count = 0;
    if (!isError && !robot_is_stop)
    {
      ostringstream oss;
      oss.str("");
      oss << "buffer size 大于阈值，停止移动";
      syslog(LOG_INFO, oss.str().c_str());
      std_msgs::Int32 robot_status;
      robot_status.data = 0;
      control_pub.publish(robot_status);
      isError = true;
      robot_is_stop = true;
    }
    if (msg->data >= clear_buff_size && isError && !hdl_is_sleep)
    {
      clear_count++;
      ostringstream oss;
      oss.str("");
      oss << "clear buffer 第 :" << clear_count << "次";
      syslog(LOG_INFO, oss.str().c_str());
      std_msgs::Bool clear_buffer_status;
      clear_buffer_status.data = true;
      clear_buff_pub.publish(clear_buffer_status);
      if (clear_count >= threshold_clear_count)
      {
        //机器人运算不足后触发停止定位程序
        ostringstream oss;
        oss.str("");
        oss << "clear_buff_size  > :" << threshold_clear_count << "stop hdl >< >< ><";
        syslog(LOG_INFO, oss.str().c_str());
        // std_msgs::Bool hdl_status;
        // hdl_status.data = true;
        // stop_hdl_pub.publish(hdl_status);
        set_hdl_status(true);
        //计时一分钟
        sleep_time = ros::Time::now();
        hdl_is_sleep = true;
      }
    }
    syslog(LOG_INFO, "********************buffer size warnning************************");
    syslog(LOG_INFO, "position :(%d,%d,%d)", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
    syslog(LOG_INFO, "yd_distance offset:%d", yd_distance);
    syslog(LOG_INFO, "euler angle offset:(%d,%d,%d)", (roll_ - roll), (pitch_ - pitch), (yaw_ - yaw));
    syslog(LOG_INFO, "clear buffer times: %d", clear_count);
  }
  else
  {
    laster_manual_count++;
    if (isError && robot_is_stop)
    {
      clear_count = 0;
      ostringstream oss;
      oss.str("");
      oss << "buffer size 小于阈值，恢复移动";
      syslog(LOG_INFO, oss.str().c_str());
      std_msgs::Int32 status;
      status.data = 1;
      control_pub.publish(status);
      isError = false;
      robot_is_stop = false;
    }
  }
  //std::cout << "laster_manual_count " << laster_manual_count << " laster_recover_count" << laster_recover_count << std::endl;
}

int main(int argc, char **argv)
{
  //init log
  openlog("monitor", LOG_PID, LOG_LOCAL1);

  ros::init(argc, argv, "monitor");
  monitor_ros_node node;
  ROS_INFO("monitor ros node started...");

  ros::Rate rate(10);
  while (ros::ok())
  {
    node.update();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
