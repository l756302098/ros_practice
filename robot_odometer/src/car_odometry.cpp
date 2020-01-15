#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>

#include <string>
#include <cstdio>

float wheelbase = 0.5;

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    //int
    float wheel_left = 600;
    float wheel_right = 600;
    float vl = 0, vr = 0,v = 0,w = 0;
    vl = wheel_left / 30;
    vr = wheel_right / 30;
    v = (vl + vr) / 2;
    w = (vr - vl) / wheelbase;
    std::cout << "velocity:" << v << " angulr:" << w << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_odometer");

    ros::NodeHandle nh;
    ROS_INFO("robot_odometer node started ...");
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

    ros::Rate rate(10);
    while (ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}