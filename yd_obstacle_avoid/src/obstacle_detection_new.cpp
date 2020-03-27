/*
1.获取机器人的位置姿态
2.获取局部地图
3.在局部地图中找出道路
4.在道路上检测是否有遮挡机器人的障碍物
5.提供服务，用于查询避过障碍物可到达的坐标点
*/
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "std_msgs/Float32.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yidamsg/task_status.h>
#include <yd_obstacle_avoid/obstacle_detection_new.h>

namespace yd_obstacle_avoid
{

using namespace std;
using namespace Eigen;

typedef Matrix<float, 4, 1> Vector4f;

obstacle_detection_new::obstacle_detection_new(/* args */)
{
    nh.param<float>("/intelligent_plan_node/robot_width", robot_width, 0.6);
    nh.param<float>("/intelligent_plan_node/detection_length", detection_length, 2.0);
    nh.param<float>("/intelligent_plan_node/edge", edge, 0.8);
    nh.param<bool>("/intelligent_plan_node/is_pub_road", is_pub_road, true);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/yida/obstacle_avoid/road", 1);
    new_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/yida/obstacle_avoid/goal", 1);
    cout << "robot_width:" << robot_width << endl;
    cout << "detection_length:" << detection_length << endl;
    cout << "edge:" << edge << endl;
    cout << "is_pub_road:" << is_pub_road << endl;
}

obstacle_detection_new::~obstacle_detection_new()
{
}

int obstacle_detection_new::get_grid_value(unsigned int &g_x, unsigned int &g_y)
{
}

int obstacle_detection_new::abs(unsigned int a, unsigned int b)
{
    if (a >= b)
    {
        return a - b;
    }
    else
    {
        return b - a;
    }
}

bool obstacle_detection_new::detection(float &obs_dis)
{
    ROS_DEBUG_STREAM_THROTTLE(1,"start detection ...");
    //sim
    // alldis = 5;
    // dis = 3;
    //判断位置
    if (!is_add_map)
    {
        ROS_DEBUG_STREAM_THROTTLE(1,"waitting for map");
        return false;
    }
    //判断地图
    if (!is_had_pos)
    {
        ROS_DEBUG_STREAM_THROTTLE(1, "waitting for robot pos");
        return false;
    }
    remdis = alldis - dis;
    //判断是否符合条件
    //1.去除原地旋转的条件 离起点/终点>0.5m
    
    if (alldis == 0 || dis == 0)
    {
        ROS_DEBUG_STREAM_THROTTLE(1, "waitting for task status");
        return false;
    }
    if (alldis < (2 * edge + robot_width))
    {
        ROS_DEBUG("Road is too short edge:%.2f alldis:%.2f robot_width:%.2f", edge, alldis, robot_width);
        return false;
    }
    if (dis < edge)
    {
        ROS_DEBUG("Too close to starting point　alldis:%.2f dis:%.2f edge:%.2f", alldis, dis, edge);
        return false;
    }
    if (remdis < edge)
    {
        ROS_DEBUG("Too close to the end　alldis:%.2f dis:%.2f edge:%.2f", alldis, dis, edge);
        return false;
    }
    if (direct==0)
    {
        ROS_DEBUG_STREAM_THROTTLE(1, "The car is reversing");
        return false;
    }

    if (!worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, center_x, center_y))
    {
        ROS_ERROR("0 world to map error %i/%i", center_x, center_y);
        return false;
    }
    
    //从当前位置向前延伸一定距离(radiu),判断是否有障碍物
    //先确定到终点的距离和检测距离的大小
    float final_length = remdis > detection_length ? detection_length : remdis;

    bool is_obs = false;
    //先左右扩展,在前后扩展，然后对应到像素
    Eigen::Quaterniond q(robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
    //forward
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d f = q * v;
    //rotate
    Eigen::AngleAxisd QX90(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond t_Q(QX90);
    Eigen::Vector3d r = q * t_Q * v;
    // calc left down
    Eigen::Vector3d temp_pos(0, 0, 0);
    temp_pos[0] = robot_pose.pose.position.x - r[0] * robot_width / 2;
    temp_pos[1] = robot_pose.pose.position.y - r[1] * robot_width / 2;
    temp_pos[2] = robot_pose.pose.position.z - r[2] * robot_width / 2;

    nav_msgs::OccupancyGrid obs_map;
    obs_map.header.frame_id = "map";
    obs_map.header.stamp = ros::Time::now();
    obs_map.info = grid->info; // uint32
    int total = grid->info.width * grid->info.height;
    int p[total] = {0};
    std::vector<signed char> a(p, p + total);
    obs_map.data = a;
    int cell_width = robot_width / resolution_;
    int cell_height = final_length / resolution_;
    unsigned int cell_center_x, cell_center_y;
    for (int i = 0; i < cell_width; i++)
    {
        double cell_pox_x = temp_pos[0] + r[0] * resolution_ * i;
        double cell_pox_y = temp_pos[1] + r[1] * resolution_ * i;

        if (!worldToMap(cell_pox_x, cell_pox_y, cell_center_x, cell_center_y))
        {
            ROS_WARN("1 world to map error %i/%i", cell_center_x, cell_center_y);
            //cout << "1 world to map error " << cell_center_x << "/" << cell_center_y << endl;
            continue;
        }
        if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
        {
            //is_obs = true;
            //ROS_INFO("Obstacles detected on road ahead!!!");

            //break;
        }
        for (int j = 3; j < cell_height; j++)
        {
            double cell_pox_x_ = cell_pox_x + f[0] * resolution_ * j;
            double cell_pox_y_ = cell_pox_y + f[1] * resolution_ * j;
            if (!worldToMap(cell_pox_x_, cell_pox_y_, cell_center_x, cell_center_y))
            {
                ROS_WARN("2 world to map error %i/%i", cell_center_x, cell_center_y);
                continue;
            }
            obs_map.data[cell_center_x + cell_center_y * size_x_] = 100;
            if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
            {
                ROS_INFO("Obstacles detected on road ahead!!!");
                is_obs = true;
                break;
            }
        }
    }
    //reset task status
    reset();
    if (is_pub_road)
        map_pub.publish(obs_map);
    if (is_obs)
    {
        float distance_x = 0, distance_y = 0, distance = 0;
        int cell_dis_x = abs(cell_center_x, center_x);
        distance_x = cell_dis_x * resolution_;
        int cell_dis_y = abs(cell_center_y, center_y);
        distance_y = cell_dis_y * resolution_;
        distance = sqrt(distance_x * distance_x + distance_y * distance_y);
        obs_dis = distance;
        ROS_INFO("Obstacles %.4f meters in front!", distance);
        return true;
    }
    else
    {
        ROS_DEBUG("No Obstacles in front!");
        obs_dis = 0;
        return false;
    }
}

bool obstacle_detection_new::calc_new_goal(float distance, geometry_msgs::PoseStamped &goal)
{
    //sim
    // alldis = 5;
    // dis = 3;
    ROS_DEBUG("start calc new goal ...");
    //判断位置
    if (!is_add_map || !is_had_pos)
    {
        ROS_DEBUG("Can't calculate new goal: waitting for map");
        return false;
    }
    //在检测范围之外寻找一个新的目标点/pos
    //前提:总长大于2m且剩余大于1m
    //1.越过障碍物
    //2.在机器人的正前方
    //3.不能超过有效距离，在终点之后,在地图的半径之内
    
    if (alldis_ == 0 || dis_ == 0)
    {
        ROS_DEBUG("waitting for task status");
        return false;
    }
    remdis_ = alldis_ - dis_;
    if ((distance + robot_width) > remdis_)
    {
        ROS_DEBUG("Can't calculate: too close to the end");
        ROS_DEBUG_STREAM("alldis:" << alldis_ << " robot_width:"
                                   << robot_width << " dis:" << dis_ << " distance:" << distance);
        return false;
    }
    
    float map_radiu = size_x_ * resolution_ / 2.0;
    float max_distance = min(remdis_, map_radiu);

    if (distance > max_distance)
    {
        ROS_DEBUG("Can't calculate: Obstacles have exceeded the safe area");
        ROS_DEBUG_STREAM("alldis:" << alldis_ << " max_distance:"
                                   << max_distance << " remdis:" << remdis_ << " distance:" << distance);
        return false;
    }
    if (direct == 0)
    {
        ROS_INFO_STREAM_THROTTLE(1, "The car is reversing");
        return false;
    }

    float min_distance = distance;

    //start calc
    if (!worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, center_x, center_y))
    {
        ROS_ERROR("00 world to map error in %i / %i", center_x, center_y);
        //cout << "00 world to map error " << center_x << "/" << center_y << endl;
        return false;
    }
    int min_cell = min_distance / resolution_;
    int max_cell = max_distance / resolution_;
    int robot_cell = robot_width / resolution_;
    max_cell = max_cell - robot_cell;
    bool is_find = false;
    Eigen::Quaterniond q(robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
    //forward
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d f = q * v;
    unsigned int cell_center_x = 0, cell_center_y = 0;
    int k = 0;
    ROS_DEBUG("min_cell:%i  max_cell:%i", min_cell, max_cell);
    for (int i = max_cell; i > min_cell + robot_cell; i--)
    {
        //找出连续没有障碍物的一段，取中间值
        for (int j = 0; j < robot_cell; j++)
        {
            k = i - j;
            double cell_pox_x = robot_pose.pose.position.x + f[0] * resolution_ * k;
            double cell_pox_y = robot_pose.pose.position.y + f[1] * resolution_ * k;
            if (!worldToMap(cell_pox_x, cell_pox_y, cell_center_x, cell_center_y))
            {
                //ROS_ERROR("01 world to map error in %i / %i", cell_center_x, cell_center_y);
                ROS_WARN_STREAM("01 world to map error " << cell_center_x << "/" << cell_center_y);
                continue;
            }
            //判断是否有占用
            if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
            {
                //cout << "Map is occupied in x:" << cell_center_x << " y:" << cell_center_y << endl;
                continue;
            }
            ROS_DEBUG("cell_center_x:%i  cell_center_y:%i", cell_center_x, cell_center_y);
            if (j == robot_cell - 1)
            {
                k = i;
                ROS_DEBUG("Found!");
                is_find = true;
                i -= max_cell;
                break;
            }
        }
    }

    if (is_find)
    {
        ROS_INFO("found new goal! K:%i", k);
        //k = k - robot_cell / 2;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";

        goal.pose.position.x = robot_pose.pose.position.x + f[0] * resolution_ * k;
        goal.pose.position.y = robot_pose.pose.position.y + f[1] * resolution_ * k;
        goal.pose.position.z = robot_pose.pose.position.z;
        goal.pose.orientation.x = robot_pose.pose.orientation.x;
        goal.pose.orientation.y = robot_pose.pose.orientation.y;
        goal.pose.orientation.z = robot_pose.pose.orientation.z;
        goal.pose.orientation.w = robot_pose.pose.orientation.w;
        
        new_goal_pub.publish(goal);

        return true;
    }
    else
    {
        ROS_INFO("not found new goal!");
        return false;
    }
}

void obstacle_detection_new::set_map(const nav_msgs::OccupancyGrid::Ptr map)
{
    //cout << "set_map" << endl;
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;
    size_x_ = map->info.width;
    size_y_ = map->info.height;
    grid = map;
    is_add_map = true;
}

bool obstacle_detection_new::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const
{
    if (wx < origin_x_ || wy < origin_y_)
    {
        ROS_ERROR("wx/wy %.2f / %.2f origin_x_/origin_y_ %.2f / %.2f", wx, wy, origin_x_, origin_y_);
        return false;
    }

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
    {
        return true;
    }
    return false;
}

void obstacle_detection_new::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    Quaternionf quanternion = Quaternionf(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    robot_qua << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

    robot_pose.pose.position.x = pose_msg->pose.pose.position.x;
    robot_pose.pose.position.y = pose_msg->pose.pose.position.y;
    robot_pose.pose.position.z = pose_msg->pose.pose.position.z;
    robot_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    robot_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    robot_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    robot_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

    is_had_pos = true;
}

double obstacle_detection_new::radian_to_angle(double radian)
{
    return 180 * radian / M_PI;
}

void obstacle_detection_new::task_status(const yidamsg::task_status::Ptr msg)
{
    ROS_DEBUG("get task status");
    //总长度
    alldis = msg->alldis;
    //已经行走的长度
    dis = msg->dis;
    direct = msg->direction;
    alldis_ = alldis;
    dis_ = dis;
    direct_ = direct;
}

void obstacle_detection_new::new_goal(const std_msgs::Float32::ConstPtr &msg)
{
    float distance = msg->data;
    ROS_INFO_STREAM("obstacle_detection_new calc new goal,distance " << distance);
    geometry_msgs::PoseStamped new_goal;
    if (calc_new_goal(distance, new_goal))
    {
        ROS_INFO("calc new goal success!");
    }
    else
    {
        ROS_INFO("calc new goal failed!");
    }
}
void obstacle_detection_new::reset()
{
    ROS_DEBUG_STREAM("obstacle_detection_new reset");
    alldis = 0;
    dis = 0;
    direct = 1;
}
}