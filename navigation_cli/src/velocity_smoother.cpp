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
#include <c2_algorithm.h>
#include <velocity_smoother.h>

velocity_smoother::velocity_smoother()
{
    nh.param<double>("/velocity_smoother_node/v_max_vel", v_max_vel, 0.3);
    nh.param<double>("/velocity_smoother_node/v_min_vel", v_min_vel, 0.0);
    nh.param<double>("/velocity_smoother_node/v_max_acc", v_max_acc, 0.0);
    nh.param<double>("/velocity_smoother_node/v_min_acc", v_min_acc, 0.0);
    nh.param<double>("/velocity_smoother_node/v_jeck", v_jeck, 0.0);

    nh.param<double>("/velocity_smoother_node/a_max_vel", a_max_vel, 0.3);
    nh.param<double>("/velocity_smoother_node/a_min_vel", a_min_vel, 0.0);
    nh.param<double>("/velocity_smoother_node/a_max_acc", a_max_acc, 0.0);
    nh.param<double>("/velocity_smoother_node/a_min_acc", a_min_acc, 0.0);
    nh.param<double>("/velocity_smoother_node/a_jeck", a_jeck, 0.0);

    nh.param<std::string>("/velocity_smoother_node/raw_cmd_vel_topic", raw_vel_topic, "");
    nh.param<std::string>("/velocity_smoother_node/smooth_cmd_vel_topic", smooth_vel_topic, ""); //feedback_topic
    nh.param<std::string>("/velocity_smoother_node/robot_feedback_topic", feedback_topic, "");

    cout << "smooth_vel_topic:" << smooth_vel_topic << endl;
    cout << "raw_vel_topic:" << raw_vel_topic << endl;
    cout << "feedback_topic:" << feedback_topic << endl;
    cout << "v_max_vel:" << v_max_vel << endl;
    cout << "v_min_vel:" << v_min_vel << endl;
    cout << "v_max_acc:" << v_max_acc << endl;
    cout << "v_min_acc:" << v_min_acc << endl;
    cout << "v_jeck:" << v_jeck << endl;

    v_last_pos = 0;
    v_last_vel = 0;
    v_last_acc = 0;

    a_last_acc = 0;
    a_last_pos = 0;
    a_last_vel = 0;
    vel_c2.init(v_min_vel, v_max_vel, v_min_acc, v_max_acc, v_jeck, 100);
    ang_c2.init(a_min_vel, a_max_vel, a_min_acc, a_max_acc, a_jeck, 100);

    cmd_pub = nh.advertise<geometry_msgs::Twist>(smooth_vel_topic, 1, true);
    vel_sub = nh.subscribe(raw_vel_topic, 1, &velocity_smoother::velocity_cb, this);
    if (!feedback_topic.empty())
    {
        ROS_INFO("node add subscribe feedback_topic");
        odom_sub = nh.subscribe(feedback_topic, 1, &velocity_smoother::odom_cb, this);
    }
}

velocity_smoother ::~velocity_smoother()
{
}

void velocity_smoother::velocity_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_DEBUG_STREAM("get raw x:" << v_t_vel << " z:" << a_t_vel << endl);
    //cout << "get raw x:" << v_t_vel << " z:" << a_t_vel << endl;
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

void velocity_smoother::odom_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    mutex.lock();
    v_last_vel = (msg->twist.linear.x) / 1000;
    a_last_vel = (msg->twist.angular.z)/1000;
    mutex.unlock();
}

void velocity_smoother::update()
{
    //calc twist
    mutex.lock();

    vel_c2.get_qk(v_last_acc, v_last_vel, v_acc, v_vel);
    v_last_pos = v_pos;
    //v_last_vel = v_vel;
    v_last_acc = v_acc;

    ang_c2.get_qk(a_last_acc, a_last_vel, a_acc, a_vel);
    a_last_pos = a_pos;
    //a_last_vel = a_vel;
    a_last_acc = a_acc;
    if (feedback_topic.empty()){
        v_last_vel = v_vel;
        a_last_vel = a_vel;
    }
    mutex.unlock();


    geometry_msgs::Twist twist;
    twist.linear.x = v_vel;
    twist.angular.z = a_vel;
    cmd_pub.publish(twist);
    ROS_DEBUG_STREAM("deal smooth x:" << v_vel << " z:" << a_vel << endl);
}
void velocity_smoother::reset()
{
    v_last_vel = 0;
    v_last_acc = 0;
    a_last_vel = 0;
    a_last_acc = 0;
    v_t_vel = 0;
    v_t_acc = 0;
    a_t_vel = 0;
    a_t_acc = 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_smoother_node");

    velocity_smoother vs;

    //set frequency
    ros::Rate rate(100);

    while (ros::ok())
    {
        vs.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

