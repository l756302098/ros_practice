#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "c3_algorithm/c2_algorithm.h"

using namespace std;

class c3_class
{
private:
    boost::mutex mutex;
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber vel_sub;
    //vel
    double v_max_vel, v_min_vel, v_max_acc, v_min_acc, v_jeck;
    double v_last_pos, v_last_vel, v_last_acc, v_pos, v_vel, v_acc, v_t_pos, v_t_vel, v_t_acc;
    double a_max_vel, a_min_vel, a_max_acc, a_min_acc, a_jeck;
    double a_last_pos, a_last_vel, a_last_acc, a_pos, a_vel, a_acc, a_t_pos, a_t_vel, a_t_acc;
    c2_algorithm vel_c2, ang_c2;

public:
    int frequency;
    std::string vel_topic, raw_topic;
    c3_class();
    ~c3_class();
    void velocity_cb(const geometry_msgs::Twist::ConstPtr &msg);
    void update();
};

c3_class::c3_class()
{
    nh.param<int>("/c3/frequency", frequency, 100);
    nh.param<double>("/c3/v_max_vel", v_max_vel, 0.3);
    nh.param<double>("/c3/v_min_vel", v_min_vel, 0.0);
    nh.param<double>("/c3/v_max_acc", v_max_acc, 0.0);
    nh.param<double>("/c3/v_min_acc", v_min_acc, 0.0);
    nh.param<double>("/c3/v_jeck", v_jeck, 0.0);

    nh.param<double>("/c3/a_max_vel", a_max_vel, 0.3);
    nh.param<double>("/c3/a_min_vel", a_min_vel, 0.0);
    nh.param<double>("/c3/a_max_acc", a_max_acc, 0.0);
    nh.param<double>("/c3/a_min_acc", a_min_acc, 0.0);
    nh.param<double>("/c3/a_jeck", a_jeck, 0.0);
    nh.param<std::string>("/c3/raw_topic", raw_topic, "");
    nh.param<std::string>("/c3/vel_topic", vel_topic, "");

    cout << "vel_topic:" << vel_topic << endl;
    cout << "raw_topic:" << raw_topic << endl;
    cout << "frequency:" << frequency << endl;
    cout << "v_max_vel:" << v_max_vel << endl;
    cout << "v_min_vel:" << v_min_vel << endl;
    cout << "v_max_acc:" << v_max_acc << endl;
    cout << "v_min_acc:" << v_min_acc << endl;
    cout << "v_jeck:" << v_jeck << endl;

    cmd_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, 1, true);
    vel_sub = nh.subscribe(raw_topic, 1, &c3_class::velocity_cb, this);

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

void c3_class::velocity_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    mutex.lock();
    //set linear
    v_pos = 0;
    v_t_pos = (v_last_vel + msg->linear.x) / 2 * 0.1;
    v_vel = v_last_vel;
    v_t_vel = msg->linear.x;
    v_acc = v_last_acc;
    v_t_acc = 0;
    vel_c2.start(v_vel, v_t_vel, v_acc, v_t_acc);
    //set angular
    a_pos = 0;
    a_t_pos = (a_last_vel + msg->angular.z) * 0.1;
    a_vel = a_last_vel;
    a_t_vel = msg->angular.z;
    a_acc = a_last_acc;
    a_t_acc = 0;
    ang_c2.start(a_vel, a_t_vel, a_acc, a_t_acc);
    mutex.unlock();
    cout << "get raw x:" << v_t_vel << " z:" << a_t_vel << endl;
}

void c3_class::update()
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
    geometry_msgs::Twist motor_control;
    if (abs(v_vel) < 0.01)
    {
        v_vel = 0;
    }
    if (abs(a_vel) < 0.01)
    {
        a_vel = 0;
    }
    motor_control.linear.x = v_vel;
    motor_control.angular.z = a_vel;
    cmd_pub.publish(motor_control);
    cout << "deal smooth x:" << v_vel << " z:" << a_vel << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_node_c3");

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