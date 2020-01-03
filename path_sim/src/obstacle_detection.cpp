/*
1.获取机器人的位置姿态
2.获取局部地图
3.在局部地图中找出道路
4.在道路上检测是否有遮挡机器人的障碍物(特定路段)
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
#include <yidamsg/task_status.h>
#include <yidamsg/get_goal.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Vect.cpp"
#include "utils.cpp"

using namespace std;
using namespace Eigen;

typedef Matrix<float, 4, 1> Vector4f;

class obstacle_detection
{
private:
    /* param */
    float robot_width, detection_length, road_min;
    /* ros node */
    string localmap_topic, odom_topic;
    ros::NodeHandle nh;
    ros::Publisher map_pub, right_pub, detection_pub;
    ros::Subscriber pose_sub, map_sub, task_sub;
    ros::ServiceServer goal_srv;
    /* map */
    uint32_t size_x_, size_y_;
    double origin_x_, origin_y_, resolution_;
    nav_msgs::OccupancyGridPtr grid;
    bool isAddMap, isHadPos;
    /* robot data */
    unsigned int center_x, center_y;
    Vector4f robot_qua;
    float alldis, dis, movedis, edge;
    /* data */
    geometry_msgs::PoseStamped robot_pose;
    unsigned int left_down_p_x, left_down_p_y, right_down_p_x, right_down_p_y,
        left_upper_p_x, right_upper_p_x;
    /* callback */
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void set_map(const nav_msgs::OccupancyGrid::Ptr map);
    void task_status(const yidamsg::task_status::Ptr msg);
    bool calc_goal(yidamsg::get_goal::Request &req, yidamsg::get_goal::Response &res);

public:
    int frequency;
    obstacle_detection(/* args */);
    ~obstacle_detection();
    void update();
    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const;
    double radian_to_angle(double radian);
    int abs(unsigned int a, unsigned int b);
    int get_grid_value(unsigned int &g_x, unsigned int &g_y);
    void publishZero(float distance = 0);
};

obstacle_detection::obstacle_detection(/* args */)
{
    nh.param<int>("/obstacle_detection/frequency", frequency, 5);
    nh.param<float>("/obstacle_detection/robot_width", robot_width, 0.6);
    nh.param<float>("/obstacle_detection/detection_length", detection_length, 2.0);
    nh.param<float>("/obstacle_detection/road_min", road_min, 5.0);
    nh.param<string>("/obstacle_detection/localmap_topic", localmap_topic, "/move_base/global_costmap/costmap");
    nh.param<string>("/obstacle_detection/odom_topic", odom_topic, "/odom_localization");
    cout << "robot_width:" << robot_width << endl;
    cout << "detection_length:" << detection_length << endl;
    cout << "road_min:" << road_min << endl;
    cout << "odom_topic:" << odom_topic << endl;
    cout << "localmap_topic:" << localmap_topic << endl;
    // localmap_topic = "/move_base/global_costmap/costmap";
    // odom_topic = "/odom_localization";

    pose_sub = nh.subscribe(odom_topic, 1, &obstacle_detection::pose_callback, this);
    map_sub = nh.subscribe(localmap_topic, 1, &obstacle_detection::set_map, this);
    //map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/test/gridMap", 1);
    //right_pub = nh.advertise<nav_msgs::Odometry>("/path_sim/right", 1, false);
    detection_pub = nh.advertise<std_msgs::Float32>("/yida/robot/obstacle_distance", 1, false);
    task_sub = nh.subscribe("/task_status", 1, &obstacle_detection::task_status, this);
    goal_srv = nh.advertiseService("/yida/obstacle/new_goal", &obstacle_detection::calc_goal, this);
}

obstacle_detection::~obstacle_detection()
{
}

int obstacle_detection::get_grid_value(unsigned int &g_x, unsigned int &g_y)
{
}

bool obstacle_detection::calc_goal(yidamsg::get_goal::Request &req, yidamsg::get_goal::Response &res)
{
    cout << "calc new goal" << endl;
    //判断位置
    if (!isAddMap || !isHadPos)
    {
        cout << "waitting for map" << endl;
        res.success = false;
        return true;
    }
    //在检测范围之外寻找一个新的目标点/pos
    //1.越过障碍物
    //2.在机器人的正前方
    //3.不能超过有效距离，在终点之后,在地图的半径之内
    float distance = req.distance;
    edge = detection_length - 0.5;
    if (edge > dis)
    {
        ROS_INFO("Can't calculate: too close to the end");
        res.success = false;
        return true;
    }
    float map_radiu = size_x_ * resolution_ / 2.0;
    float max_distance = min(dis, map_radiu);
    //float max_distance = map_radiu;
    if (distance > max_distance)
    {
        ROS_INFO("Can't calculate: Obstacles have exceeded the safe area");
        res.success = false;
        return true;
    }
    float min_distance = distance;

    //start calc
    if (!worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, center_x, center_y))
    {
        ROS_ERROR("00 world to map error in %i / %i", center_x, center_y);
        //cout << "00 world to map error " << center_x << "/" << center_y << endl;
        res.success = false;
        return true;
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
    ROS_INFO("min_cell:%i  max_cell:%i",min_cell,max_cell);
    for (int i = min_cell; i < max_cell; i++)
    {
        //找出连续没有障碍物的一段，取中间值
        for (int j = 0; j < robot_cell; j++)
        {
            k = i + j;
            double cell_pox_x = robot_pose.pose.position.x + f[0] * resolution_ * k;
            double cell_pox_y = robot_pose.pose.position.y + f[1] * resolution_ * k;
            if (!worldToMap(cell_pox_x, cell_pox_y, cell_center_x, cell_center_y))
            {
                ROS_ERROR("01 world to map error in %i / %i", cell_center_x, cell_center_y);
                //cout << "01 world to map error " << cell_center_x << "/" << cell_center_y << endl;
                continue;
            }
            //判断是否有占用
            if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
            {
                //cout << "Map is occupied in x:" << cell_center_x << " y:" << cell_center_y << endl;
                continue;
            }
            if (j == robot_cell - 1)
            {
                is_find = true;
                break;
            }
        }
    }
    
    if (is_find)
    {
        k = k - robot_cell / 2;
        res.pos_x = robot_pose.pose.position.x + f[0] * resolution_ * k;
        res.pos_y = robot_pose.pose.position.y + f[1] * resolution_ * k;
        res.pos_z = robot_pose.pose.position.z;
        //rotate
        res.qua_x = robot_pose.pose.orientation.x;
        res.qua_y = robot_pose.pose.orientation.y;
        res.qua_z = robot_pose.pose.orientation.z;
        res.qua_w = robot_pose.pose.orientation.w;

        //cout << "pos x:" << res.pos_x << " y:" << res.pos_y << " z:" << res.pos_z << endl;
        //cout << "qua x:" << res.qua_x << " y:" << res.qua_y << " z:" << res.qua_z << " w:" << res.qua_w << endl;

        res.success = true;
    }
    else
    {
        res.success = false;
    }

    return true;
}

int obstacle_detection::abs(unsigned int a, unsigned int b)
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

void obstacle_detection::publishZero(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    detection_pub.publish(msg);
}

void obstacle_detection::update()
{
    //判断位置
    if (!isAddMap)
    {
        ROS_INFO("waitting for map");
        //cout << "waitting for map" << endl;
        publishZero();
        return;
    }
    //判断地图
    if (!isHadPos)
    {
        ROS_INFO("waitting for robot pos");
        //cout << "waitting for robot pos" << endl;
        publishZero();
        return;
    }
    //判断是否符合条件
    //1.道路总长大于５ｍ
    //2.道路距离两头的距离大于 length - 0.5m
    if (alldis == 0 && dis == 0)
    {
        ROS_INFO("waitting for task status");
        //cout << "waitting for task status" << endl;
        publishZero();
        return;
    }
    if (alldis < road_min)
    {
        ROS_INFO("road is too short , road min value is:%.2f",road_min);
        //cout << "road is too short , road min value is:" << road_min << endl;
        publishZero();
        return;
    }
    movedis = alldis - dis;
    edge = detection_length - 0.5;
    if (movedis < edge)
    {
        ROS_INFO("Too close to starting point　alldis:%.2f dis:%.2f edge",alldis,dis,edge);
        //cout << "Too close to starting point　alldis：" << alldis << " dis:" << dis << "edge" << edge << endl;
        publishZero();
        return;
    }
    if (dis < edge)
    {
        ROS_INFO("Too close to the end　alldis:%.2f dis:%.2f edge", alldis, dis, edge);
        //cout << "Too close to the end　alldis：" << alldis << " dis:" << dis << "edge" << edge << endl;
        publishZero();
        return;
    }

    if (!worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, center_x, center_y))
    {
        ROS_ERROR("0 world to map error %i/%i", center_x, center_y);
        //cout << "0 world to map error " << center_x << "/" << center_y << endl;
        publishZero();
        return;
    }

    //cout << "local to map " << center_x << "/" << center_y << endl;
    //obs_map.data[center_x  + center_y * size_x_] = 100;
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

    // nav_msgs::Odometry rodom;
    // rodom.header.frame_id = "map";
    // rodom.header.stamp = ros::Time::now();
    // rodom.pose.pose.position.x = robot_pose.pose.position.x - r[0] * robot_width / 2;
    // rodom.pose.pose.position.y = robot_pose.pose.position.y - r[1] * robot_width / 2;
    // rodom.pose.pose.position.z = robot_pose.pose.position.z - r[2] * robot_width / 2;

    //right_pub.publish(rodom);

    /*
    //test
    nav_msgs::OccupancyGrid obs_map;
    obs_map.header.frame_id = "map";
    obs_map.header.stamp = ros::Time::now();
    obs_map.info = grid->info; // uint32
    int p[grid->info.width * grid->info.height] = {0};
    std::vector<signed char> a(p, p + 2500);
    obs_map.data = a;
    int cell_width = robot_width / resolution_;
    int cell_height = detection_length / resolution_;
    unsigned int cell_center_x, cell_center_y;
    for (int i = 0; i < cell_width; i++)
    {
        double cell_pox_x = temp_pos[0] + r[0] * resolution_ * i;
        double cell_pox_y = temp_pos[1] + r[1] * resolution_ * i;

        if (!worldToMap(cell_pox_x, cell_pox_y, cell_center_x, cell_center_y))
        {
            cout << "world to map error " << cell_center_x << "/" << cell_center_y << endl;
            continue;
        }
        obs_map.data[cell_center_x  + cell_center_y* size_x_] = 100;
        for (int j = 0; j < cell_height; j++)
        {
            double cell_pox_x_ = cell_pox_x + f[0] * resolution_ * j;
            double cell_pox_y_ = cell_pox_y + f[1] * resolution_ * j;
            if (!worldToMap(cell_pox_x_, cell_pox_y_, cell_center_x, cell_center_y))
            {
                cout << "world to map error " << cell_center_x << "//" << cell_center_y << endl;
                continue;
            }
            obs_map.data[cell_center_x + cell_center_y * size_x_] = 100;
        }
    }

    map_pub.publish(obs_map);
    */
    int cell_width = robot_width / resolution_;
    int cell_height = detection_length / resolution_;
    unsigned int cell_center_x, cell_center_y;
    for (int i = 0; i < cell_width; i++)
    {
        double cell_pox_x = temp_pos[0] + r[0] * resolution_ * i;
        double cell_pox_y = temp_pos[1] + r[1] * resolution_ * i;

        if (!worldToMap(cell_pox_x, cell_pox_y, cell_center_x, cell_center_y))
        {
            cout << "1 world to map error " << cell_center_x << "/" << cell_center_y << endl;
            continue;
        }
        if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
        {
            is_obs = true;
            ROS_INFO("Obstacles detected on road ahead!!!");
            //cout << "Obstacles detected on road ahead!!!" << endl;
            break;
        }
        for (int j = 2; j < cell_height; j++)
        {
            double cell_pox_x_ = cell_pox_x + f[0] * resolution_ * j;
            double cell_pox_y_ = cell_pox_y + f[1] * resolution_ * j;
            if (!worldToMap(cell_pox_x_, cell_pox_y_, cell_center_x, cell_center_y))
            {
                //cout << "2 world to map error " << cell_center_x << "//" << cell_center_y << endl;
                ROS_ERROR("2 world to map error %i/%i", cell_center_x, cell_center_y);
                continue;
            }
            if (grid->data[cell_center_x + cell_center_y * size_x_] > 0)
            {
                ROS_INFO("Obstacles detected on road ahead!!!");
                is_obs = true;
                break;
            }
        }
    }
    if (is_obs)
    {
        float distance_x = 0, distance_y = 0, distance = 0;
        int cell_dis_x = abs(cell_center_x, center_x);
        //printf("1 cell_dis_x %i　米处", cell_dis_x);
        distance_x = cell_dis_x * resolution_;
        //printf("2 distance_x %.4f　米处", distance_x);
        int cell_dis_y = abs(cell_center_y, center_y);
        distance_y = cell_dis_y * resolution_;
        //printf("distance_y %.4f　米处", distance_y);
        distance = sqrt(distance_x * distance_x + distance_y * distance_y);
        //printf("障碍物在前方 %.4f　米处", distance);
        ROS_INFO("Obstacles %.4f meters in front!", distance);
        publishZero(distance);
    }else{
        publishZero();
    }
}

void obstacle_detection::task_status(const yidamsg::task_status::Ptr msg)
{
    ROS_INFO("get task status");
    alldis = msg->alldis;
    dis = msg->dis;
}

void obstacle_detection::set_map(const nav_msgs::OccupancyGrid::Ptr map)
{
    //cout << "set_map" << endl;
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;
    size_x_ = map->info.width;
    size_y_ = map->info.height;
    //cout << "resolution:" << map->info.resolution << "origin_x:" << map->info.origin.position.x << "origin_y:" << map->info.origin.position.y << endl;
    grid = map;
    isAddMap = true;
}

bool obstacle_detection::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const
{
    if (wx < origin_x_ || wy < origin_y_)
    {
        //cout << "wx:" << wx << "wy:" << wy << endl;
        //cout << "origin_x_:" << origin_x_ << "origin_y_:" << origin_y_ << endl;
        ROS_ERROR("wx/wy %.2f / %.2f origin_x_/origin_y_ %.2f / %.2f", wx, wy, origin_x_, origin_y_);
        return false;
    }

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
    {
        return true;
    }
    //cout << "mx:" << mx << "my:" << my << endl;
    //cout << "size_x_:" << size_x_ << "size_y_:" << size_y_ << endl;
    return false;
}

void obstacle_detection::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
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

    isHadPos = true;
    //cout << "get pose " << robot_qua << endl;

    // if (!worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, center_x, center_y))
    // {
    //     cout << "world to map error " << endl;
    // }
    // cout << "mx:" << center_x << "my:" << center_y << endl;
}

double obstacle_detection::radian_to_angle(double radian)
{
    return 180 * radian / M_PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");

    obstacle_detection obd;
    ROS_INFO("obstacle_detection node started...");

    ros::Rate rate(obd.frequency);
    while (ros::ok())
    {
        obd.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
