#include "ros/ros.h"
#include <yidamsg/motor_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_verf"); //Init ROS
    ros::NodeHandle nh;

    bool is_sim = false;
    nh.param<bool>("is_sim", is_sim, false);
    std::cout << "is_sim" << is_sim << std::endl;

    ros::Publisher cmd_pub = nh.advertise<yidamsg::motor_control>("/yida/robot/motor_control", 1, true);
    ros::Rate loop_rate(10);
    ros::Duration duration = ros::Duration(10);
    ros::Time start_time = ros::Time::now();
    yidamsg::motor_control motor_control;
    while (ros::ok()){
        if (ros::Time::now() - start_time <= duration)
        {
            ROS_INFO("angular test ...");
            motor_control.control_mode = 4;
            motor_control.speed.linear.x = 0;
            motor_control.speed.angular.z = 3.14 / 180 * 18;
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