/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-02-28 11:33:51
 * @LastEditors: li
 * @LastEditTime: 2021-02-28 13:24:26
 */
#include "ros/ros.h"
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "eigen_demo/map_server.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_demo_node");
    ros::NodeHandle nh;
    map_server mp;

    ROS_INFO("eigen_demo_node started...");
    ros::Rate rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
