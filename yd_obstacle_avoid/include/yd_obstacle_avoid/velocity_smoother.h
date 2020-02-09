#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <yidamsg/motor_control.h>

#ifndef VELOCITY_SMOOTHER_VALUES_H_
#define VELOCITY_SMOOTHER_VALUES_H_

using namespace std;

class velocity_smoother
{
private:
    boost::mutex mutex;
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber vel_sub, odom_sub;
    //vel
    double v_max_vel, v_min_vel, v_max_acc, v_min_acc, v_jeck;
    double v_last_pos, v_last_vel, v_last_acc, v_pos, v_vel, v_acc, v_t_pos, v_t_vel, v_t_acc;
    double a_max_vel, a_min_vel, a_max_acc, a_min_acc, a_jeck;
    double a_last_pos, a_last_vel, a_last_acc, a_pos, a_vel, a_acc, a_t_pos, a_t_vel, a_t_acc;
    c2_algorithm vel_c2, ang_c2;

public:
    int frequency, is_use_odom;
    std::string smooth_vel_topic, raw_vel_topic, odom_topic;
    velocity_smoother();
    ~velocity_smoother();
    void velocity_cb(const geometry_msgs::Twist::ConstPtr &msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void update();
};

#endif