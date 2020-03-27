#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yidamsg/task_status.h>
#include "std_msgs/Float32.h"

#ifndef OBSTACLE_DETECTION_VALUES_H_
#define OBSTACLE_DETECTION_VALUES_H_
namespace yd_obstacle_avoid
{
using namespace std;
using namespace Eigen;
typedef Matrix<float, 4, 1> Vector4f;

class obstacle_detection
{
private:
    /* param */
    float robot_width, detection_length, road_min;
    /* ros node */
    ros::NodeHandle nh;
    ros::Publisher map_pub, right_pub, detection_pub;
    ros::Subscriber pose_sub, map_sub, task_sub;
    ros::ServiceServer goal_srv;
    /* map */
    uint32_t size_x_, size_y_;
    double origin_x_, origin_y_, resolution_;
    nav_msgs::OccupancyGridPtr grid;
    bool is_add_map, is_had_pos;
    /* robot data */
    unsigned int center_x, center_y;
    Vector4f robot_qua;
    float alldis, dis, movedis, edge;
    /* data */
    geometry_msgs::PoseStamped robot_pose;
    unsigned int left_down_p_x, left_down_p_y, right_down_p_x, right_down_p_y,
        left_upper_p_x, right_upper_p_x;

public:
    obstacle_detection(/* args */);
    ~obstacle_detection();
    bool detection(float &obs_dis);
    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const;
    double radian_to_angle(double radian);
    int abs(unsigned int a, unsigned int b);
    int get_grid_value(unsigned int &g_x, unsigned int &g_y);
    bool calc_new_goal(float distance, geometry_msgs::PoseStamped &goal);
    /* callback */
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void set_map(const nav_msgs::OccupancyGrid::Ptr map);
    void task_status(const yidamsg::task_status::Ptr msg);
    void new_goal(const std_msgs::Float32::ConstPtr &msg);
};
}
#endif