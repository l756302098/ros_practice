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
#include <std_srvs/SetBool.h>
#include <yidamsg/motor_control.h>

#include "c3_algorithm/c2_algorithm.h"

using namespace std;

class c3_class
{
private:
    boost::mutex mutex;
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber vel_sub, odom_sub;
    ros::ServiceServer smooth_srv;
    //vel
    double v_max_vel, v_min_vel, v_max_acc, v_min_acc, v_jeck;
    double v_last_pos, v_last_vel, v_last_acc, v_pos, v_vel, v_acc, v_t_pos, v_t_vel, v_t_acc;
    double a_max_vel, a_min_vel, a_max_acc, a_min_acc, a_jeck;
    double a_last_pos, a_last_vel, a_last_acc, a_pos, a_vel, a_acc, a_t_pos, a_t_vel, a_t_acc;
    c2_algorithm vel_c2, ang_c2;
    bool isSmooth;

public:
    int frequency, is_use_odom;
    std::string smooth_vel_topic, raw_vel_topic, odom_topic;
    c3_class();
    ~c3_class();
    void velocity_cb(const geometry_msgs::Twist::ConstPtr &msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    bool smooth_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void update();
    void reset();
};

c3_class::c3_class()
{
    nh.param<int>("/velocity_smoother/frequency", frequency, 100);
    nh.param<int>("/velocity_smoother/is_use_odom", is_use_odom, 0);
    nh.param<double>("/velocity_smoother/v_max_vel", v_max_vel, 0.3);
    nh.param<double>("/velocity_smoother/v_min_vel", v_min_vel, 0.0);
    nh.param<double>("/velocity_smoother/v_max_acc", v_max_acc, 0.0);
    nh.param<double>("/velocity_smoother/v_min_acc", v_min_acc, 0.0);
    nh.param<double>("/velocity_smoother/v_jeck", v_jeck, 0.0);

    nh.param<double>("/velocity_smoother/a_max_vel", a_max_vel, 0.3);
    nh.param<double>("/velocity_smoother/a_min_vel", a_min_vel, 0.0);
    nh.param<double>("/velocity_smoother/a_max_acc", a_max_acc, 0.0);
    nh.param<double>("/velocity_smoother/a_min_acc", a_min_acc, 0.0);
    nh.param<double>("/velocity_smoother/a_jeck", a_jeck, 0.0);
    nh.param<std::string>("/velocity_smoother/raw_cmd_vel_topic", raw_vel_topic, "");
    nh.param<std::string>("/velocity_smoother/smooth_cmd_vel_topic", smooth_vel_topic, ""); //odom_topic
    nh.param<std::string>("/velocity_smoother/odom_topic", odom_topic, "");

    cout << "smooth_vel_topic:" << smooth_vel_topic << endl;
    cout << "raw_vel_topic:" << raw_vel_topic << endl;
    cout << "odom_topic:" << odom_topic << endl;
    cout << "frequency:" << frequency << endl;
    cout << "v_max_vel:" << v_max_vel << endl;
    cout << "v_min_vel:" << v_min_vel << endl;
    cout << "v_max_acc:" << v_max_acc << endl;
    cout << "v_min_acc:" << v_min_acc << endl;
    cout << "v_jeck:" << v_jeck << endl;
    cout << "is_use_odom:" << is_use_odom << endl;

    smooth_srv = nh.advertiseService("/yida/robot/smooth/status", &c3_class::smooth_callback, this);
    //cmd_pub = nh.advertise<geometry_msgs::Twist>(smooth_vel_topic, 1, true);
    cmd_pub = nh.advertise<yidamsg::motor_control>(smooth_vel_topic, 1, true);
    vel_sub = nh.subscribe(raw_vel_topic, 1, &c3_class::velocity_cb, this);
    if (is_use_odom >= 1)
    {
        odom_sub = nh.subscribe(odom_topic, 1, &c3_class::odom_cb, this);
    }

    v_last_pos = 0;
    v_last_vel = 0;
    v_last_acc = 0;

    a_last_acc = 0;
    a_last_pos = 0;
    a_last_vel = 0;
    vel_c2.init(v_min_vel, v_max_vel, v_min_acc, v_max_acc, v_jeck, frequency);
    ang_c2.init(a_min_vel, a_max_vel, a_min_acc, a_max_acc, a_jeck, frequency);
}
c3_class ::~c3_class()
{
}

bool c3_class::smooth_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    //change status
    isSmooth = req.data;
    if (isSmooth)
    {
        reset();
    }
    res.success = true;
    return true;
}

void c3_class::reset()
{
    v_last_pos = 0;
    v_last_vel = 0;
}

void c3_class::velocity_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    cout << "get raw x:" << v_t_vel << " z:" << a_t_vel << endl;
    if (is_use_odom >= 1)
    {
        mutex.lock();
        //set linear
        //v_vel = v_last_vel;
        v_t_vel = msg->linear.x;
        v_acc = v_last_acc;
        v_t_acc = 0;
        vel_c2.start(v_vel, v_t_vel, v_acc, v_t_acc);
        //set angular
        //a_vel = a_last_vel;
        a_t_vel = msg->angular.z;
        a_acc = a_last_acc;
        a_t_acc = 0;
        ang_c2.start(a_vel, a_t_vel, a_acc, a_t_acc);
        mutex.unlock();
    }
    else
    {
        mutex.lock();
        //set linear
        v_vel = v_last_vel;
        v_t_vel = msg->linear.x;
        v_acc = v_last_acc;
        v_t_acc = 0;
        vel_c2.start(v_vel, v_t_vel, v_acc, v_t_acc);

        a_vel = msg->angular.z;

        //set angular
        // a_vel = a_last_vel;
        // a_t_vel = msg->angular.z;
        // a_acc = a_last_acc;
        // a_t_acc = 0;
        // ang_c2.start(a_vel, a_t_vel, a_acc, a_t_acc);
        mutex.unlock();
    }
}

void c3_class::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex.lock();
    v_vel = msg->twist.twist.linear.x;
    a_vel = msg->twist.twist.angular.z;
    mutex.unlock();
}

void c3_class::update()
{
    if (!isSmooth)
        return;
    //calc twist
    mutex.lock();
    //只对线速度做平滑，暂时不处理角速度，角速度不接受小数
    vel_c2.get_qk(v_last_acc, v_last_vel, v_acc, v_vel);
    v_last_pos = v_pos;
    v_last_vel = v_vel;
    v_last_acc = v_acc;

    // ang_c2.get_qk(a_last_acc, a_last_vel, a_acc, a_vel);
    // a_last_pos = a_pos;
    // a_last_vel = a_vel;
    // a_last_acc = a_acc;
    mutex.unlock();
    //cout << "last_pos:" << last_pos << " last_vel:" << last_vel << " last_acc:" << last_acc << endl;
    //cmd_pub
    //geometry_msgs::Twist motor_control;
    yidamsg::motor_control motor_control;
    if (abs(v_vel) < 0.01)
    {
        v_vel = 0;
    }
    if (abs(a_vel) < 0.01)
    {
        a_vel = 0;
    }
    motor_control.speed.linear.x = v_vel;
    motor_control.speed.angular.z = a_vel;
    cmd_pub.publish(motor_control);
    cout << "deal smooth x:" << v_vel << " z:" << a_vel << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_smoother");

    c3_class c3;

    //set frequency
    ros::Rate rate(c3.frequency);

    while (ros::ok())
    {
        c3.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}