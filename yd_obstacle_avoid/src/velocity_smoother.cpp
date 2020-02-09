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
#include <yd_obstacle_avoid/c2_algorithm.h>
#include <yidamsg/motor_control.h>
#include <yd_obstacle_avoid/velocity_smoother.h>

velocity_smoother::velocity_smoother()
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
    {
        odom_sub = nh.subscribe(odom_topic, 1, &velocity_smoother::odom_cb, this);
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
velocity_smoother ::~velocity_smoother()
{
}

void velocity_smoother::velocity_cb(const geometry_msgs::Twist::ConstPtr &msg)
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
        //set angular
        a_vel = a_last_vel;
        a_t_vel = msg->angular.z;
        a_acc = a_last_acc;
        a_t_acc = 0;
        ang_c2.start(a_vel, a_t_vel, a_acc, a_t_acc);
        mutex.unlock();
    }
}

void velocity_smoother::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex.lock();
    v_vel = msg->twist.twist.linear.x;
    a_vel = msg->twist.twist.angular.z;
    mutex.unlock();
}

void velocity_smoother::update()
{
    //calc twist
    mutex.lock();

    vel_c2.get_qk(v_last_acc, v_last_vel, v_acc, v_vel);
    v_last_pos = v_pos;
    v_last_vel = v_vel;
    v_last_acc = v_acc;

    ang_c2.get_qk(a_last_acc, a_last_vel, a_acc, a_vel);
    a_last_pos = a_pos;
    a_last_vel = a_vel;
    a_last_acc = a_acc;
    mutex.unlock();
    //cout << "last_pos:" << last_pos << " last_vel:" << last_vel << " last_acc:" << last_acc << endl;
    //cmd_pub
    yidamsg::motor_control motor_control;
    if (abs(v_vel) < 0.01)
    {
        v_vel = 0;
    }
    if (abs(a_vel) < 0.01)
    {
        a_vel = 0;
    }
    motor_control.control_model = 4;
    motor_control.speed.linear.x = v_vel;
    motor_control.speed.angular.z = a_vel;
    cmd_pub.publish(motor_control);
    cout << "deal smooth x:" << v_vel << " z:" << a_vel << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yd_velocity_smoother");

    velocity_smoother vs;

    //set frequency
    ros::Rate rate(vs.frequency);

    while (ros::ok())
    {
        vs.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
