#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Vect.cpp"

using namespace std;
using namespace Eigen;

typedef Matrix<float, 3, 1> Vector3f;

class PathSim
{
public:
    PathSim();
    ~PathSim();
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
    pair<double, double> calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos);
    double radian_to_angle(double radian);

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber pose_sub, goal_sub;
    geometry_msgs::PoseStamped goal, robot_pose;
    bool is_arrived;
    int pos_type, hz;
    Vect dir, forward, right;
};

PathSim ::PathSim()
{
    pose_sub = nh.subscribe("/odom_localization", 1, &PathSim::pose_callback, this);
    goal_sub = nh.subscribe("/sim/goal", 1, &PathSim::goal_callback, this);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/sim/commands/velocity", 5, true);
    is_arrived = true;
    pos_type = 0;
    hz = 15;
}

PathSim ::~PathSim()
{
}

void PathSim ::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    //save data
    robot_pose.pose.position.x = pose_msg->pose.pose.position.x;
    robot_pose.pose.position.y = pose_msg->pose.pose.position.y;
    robot_pose.pose.position.z = pose_msg->pose.pose.position.z;
    robot_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    robot_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    robot_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    robot_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
    //calc cmd
    if (!is_arrived)
    {
        geometry_msgs::Twist motor_control;
        pair<double, double> dis_ang = calculate_distance_angle(robot_pose, goal);
        if (dis_ang.first > 0.1)
        {
            if (dis_ang.second > 10)
            {
                //rotate
                if (pos_type == 1 || pos_type == 3)
                {
                    //right
                    motor_control.angular.z = -dis_ang.second * M_PI / 180 / hz;
                }
                else
                {
                    //left
                    motor_control.angular.z = dis_ang.second * M_PI / 180 / hz;
                }
            }
            else
            {
                //go straight

                motor_control.linear.x = dis_ang.first / 10 / hz;
            }
            cmd_pub.publish(motor_control);
        }
        else
        {
            is_arrived = true;
        }
    }
}

double PathSim::radian_to_angle(double radian)
{
    return 180 * radian / M_PI;
}

void PathSim ::goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
{
    goal = *goal_msg;
}

pair<double, double> PathSim::calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos)
{
    //calc robot goal relative position
    dir = Vect(goal_pos.pose.position.x - self_pos.pose.position.x, goal_pos.pose.position.y - self_pos.pose.position.y, goal_pos.pose.position.z - self_pos.pose.position.z);
    //calc forward direction
    Eigen::Quaterniond q(self_pos.pose.orientation.w, self_pos.pose.orientation.x, self_pos.pose.orientation.y, self_pos.pose.orientation.z);
    Eigen::Vector3d v(0, 1, 0);
    Eigen::Vector3d f = q * v;
    forward = Vect(f[0], f[1], f[2]);
    float dot = forward.dot(dir.normalized());
    cout << "forward:" << f << endl;
    //calc right direction
    Eigen::AngleAxisd QX90(M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond t_Q(QX90);
    Eigen::Vector3d r = t_Q * f;
    right = Vect(r[0], r[1], r[2]);
    cout << "right:" << r << endl;
    float dot1 = right.dot(dir.normalized());
    if (dot > 0)
    {
        cout << "前 ";
        if (dot1 > 0)
        {
            cout << "右 " << endl;
            pos_type = 1;
        }
        else
        {
            cout << "左 " << endl;
            pos_type = 2;
        }
    }
    else
    {
        cout << "后 ";
        if (dot1 > 0)
        {
            cout << "右 " << endl;
            pos_type = 3;
        }
        else
        {
            cout << "左 " << endl;
            pos_type = 4;
        }
    }
    //calc angle
    double angle = acos(dot) * 180 / 3.1415926;
    cout << "angle:" << angle << endl;

    //calc distance
    double distance = sqrt(pow(goal_pos.pose.position.x - self_pos.pose.position.x, 2) + pow(goal_pos.pose.position.y - self_pos.pose.position.y, 2));
    return make_pair(distance, angle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_sim");
    ros::start();

    PathSim ps;
    ros::spin();

    ros::shutdown();
    return 0;
}