#include "ros/ros.h"
#include <yidamsg/motor_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_verf"); //Init ROS
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<yidamsg::motor_control>("/yida/robot/motor_control", 1, true);
    ros::Rate loop_rate(10);
    ros::Duration duration = ros::Duration(20);
    ros::Time start_time = ros::Time::now();
    yidamsg::motor_control motor_control;
    while (ros::ok()){
        if (ros::Time::now() - start_time <= duration)
        {
            ROS_INFO("linear test ...");
            motor_control.control_mode = 4;
            motor_control.speed.linear.x = 0.1;
            motor_control.speed.angular.z = 0;
            cmd_pub.publish(motor_control);
        }else{
            motor_control.control_mode = 4;
            motor_control.speed.linear.x = 0;
            motor_control.speed.angular.z = 0;
            cmd_pub.publish(motor_control);
        }

        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};