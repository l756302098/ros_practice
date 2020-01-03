#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "stp/stp7.h"

using namespace std;

int main(int argc, char **argv)
{
    // Stp7 stp;

    // //test
    // stp.planFastestProfile(0, 30, 0, 6, 0, 4, 2);
    // try
    // {
    //     cout << "duration:" << stp.getDuration() << endl;
    //     cout << "pos:" << stp.pos(8.46) << endl;
    //     cout << "vel:" << stp.vel(8.46) << endl;
    //     cout << "acc:" << stp.acc(8.46) << endl;

    //     stp.scaleToDuration(5);
    //     cout << "pos:" << stp.pos(5.0) << endl;
    //     cout << "vel:" << stp.vel(5.0) << endl;
    //     cout << "acc:" << stp.acc(5.0) << endl;
    // }
    // catch (logic_error le)
    // {
    //     cout << le.what() << endl;
    //     cout << "Falling back to 3nd order trajectory..." << endl;
    // }
    // sleep(2);

    ros::init(argc, argv, "smooth_vel");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("test/vel", 1, true);
    Stp7 stp;
    stp.planFastestProfile(0.2, 0.1, 0.2, 2, 0, 4, 2);
    double duration = stp.getDuration();
    cout << "duration:" << duration << endl;
    ros::Rate rate(100);
    double time = 0;
    while (ros::ok())
    {
        //double secs = ros::Time::now().toSec();
        cout << "time:" << time << endl;
        geometry_msgs::Twist motor_control;
        double pos = stp.pos(time);
        double vel = stp.vel(time);
        motor_control.linear.x = pos;
        motor_control.linear.y = vel;
        cout << "pos:" << pos << endl;
        motor_control.angular.z = 0;
        cmd_pub.publish(motor_control);
        ros::spinOnce();
        rate.sleep();
        time += 0.01;
        if (time > duration + 1)
        {
            time = 0;
        }
    }

    return 0;
}