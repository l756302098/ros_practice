#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Vect.cpp"
#include "utils.cpp"

#include <math.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include <yidamsg/auto_obstacle_avoid.h>
#include <yidamsg/get_goal.h>
#include <yidamsg/obstacle_avoid_result.h>
#include <std_srvs/SetBool.h>

using namespace std;
using namespace Eigen;

typedef Matrix<float, 4, 1> Vector4f;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

enum PlanStage
{
    None,
    Await,
    Plan,
    ArriveArea,
    Finish,
};

class obstacle_avoid_planning
{
public:
    obstacle_avoid_planning();
    ~obstacle_avoid_planning();
    void path_callback(const nav_msgs::PathConstPtr &path_msg);
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
    pair<double, double> calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos);
    double radian_to_angle(double radian);
    pair<double, double> calc_twist(float angle, float distance);
    float dot(Vector3d v1, Vector3d v2);
    Vector3d normalized(Vector3d v1);
    double length(Vector3d v1);
    void deal_str(string topic, geometry_msgs::PoseStamped *goal_msg);
    void control_callback(const std_msgs::Int16::ConstPtr &msg);
    std::vector<geometry_msgs::PoseStamped> global_plan;
    void update();
    bool isArrived(double x, double y, double z);
    void sim_move(geometry_msgs::PoseStamped goal);
    void publish_new_goal();

private:
    boost::mutex path_mutex;
    ros::ServiceServer planning_srv;
    ros::ServiceClient planning_client, task_mananger_client, smooth_client;
    ros::NodeHandle nh;
    ros::Publisher cmd_pub, goal_pub, forward_pub, right_pub, avoid_goal_pub;
    ros::Subscriber pose_sub, goal_sub, path_sub, control_sub;
    geometry_msgs::PoseStamped goal, robot_pose, next_pose, a_pose, b_pose, c_pose, d_pose;
    int pos_type, current_num, frequency, follow_num;
    nav_msgs::Path path;
    Vector3d next_goal, dir, forward, right;
    Vector4f robot_qua, navigation_qua;
    std::string a_s, b_s, c_s, d_s, cmd_topic;
    static PlanStage plan_stage;
    float max_velocity, max_angular, xy_goal_tolerance, yaw_goal_tolerance;
    //move_base callback
    float obs_distance;
    bool isAvoid;
    Client *client;
    static int move_base_result;
    static void activeCb()
    {
        plan_stage = Await;
        ROS_INFO("Goal just went active");
    }
    static void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        //ROS_INFO(" feedbackCb : %f ", feedback->base_position);
    }
    static void doneCb(const actionlib::SimpleClientGoalState &state,
                       const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        if (state == state.SUCCEEDED)
        {
            ROS_INFO("success avoid obs");
            obstacle_avoid_planning::move_base_result = 1;
        }
        else
        {
            ROS_INFO("failed avoid obs");
            obstacle_avoid_planning::move_base_result = 2;
        }
    }

    //service
    bool planning_callback(yidamsg::auto_obstacle_avoid::Request &req, yidamsg::auto_obstacle_avoid::Response &res);
};

int obstacle_avoid_planning::move_base_result = 0;
PlanStage obstacle_avoid_planning::plan_stage = PlanStage::None;

bool obstacle_avoid_planning::isArrived(double x, double y, double z)
{
    double dis_x = x - goal.pose.position.x;
    double dis_y = y - goal.pose.position.y;
    double distance = sqrt(dis_x * dis_x + dis_y * dis_y);

    if (distance < xy_goal_tolerance)
        return true;
    else
    {
        return false;
    }
}

obstacle_avoid_planning ::obstacle_avoid_planning()
{
    client = new Client("move_base", true);
    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started, sending goal.");

    planning_srv = nh.advertiseService("/yida/robot/auto_obstacle_avoid", &obstacle_avoid_planning::planning_callback, this);
    planning_client = nh.serviceClient<yidamsg::get_goal>("/yida/obstacle/new_goal");
    task_mananger_client = nh.serviceClient<yidamsg::obstacle_avoid_result>("/yida/robot/obstacle_avoid_result");
    smooth_client = nh.serviceClient<std_srvs::SetBool>("/yida/robot/smooth/status");

    nh.param<std::string>("/obstacle_avoid_planning/cmd_topic", cmd_topic, "/mobile_base/commands/velocity");
    cout << cmd_topic << endl;

    // forward_pub = nh.advertise<nav_msgs::Odometry>("/path_sim/forward", 1, false);
    // right_pub = nh.advertise<nav_msgs::Odometry>("/path_sim/right", 1, false);
    path_sub = nh.subscribe("/move_base/GlobalPlanner/plan", 1, &obstacle_avoid_planning::path_callback, this);
    pose_sub = nh.subscribe("/odom_localization", 1, &obstacle_avoid_planning::pose_callback, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &obstacle_avoid_planning::goal_callback, this);
    control_sub = nh.subscribe<std_msgs::Int16>("/obstacle_avoid_planning/sim", 1, &obstacle_avoid_planning::control_callback, this);
    cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1, true);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, false);
    avoid_goal_pub = nh.advertise<nav_msgs::Odometry>("/yida/avoid/goal", 1, false);

    nh.param<float>("/obstacle_avoid_planning/max_velocity", max_velocity, 0.2);
    nh.param<float>("/obstacle_avoid_planning/max_angular", max_angular, 0.2);
    nh.param<int>("/obstacle_avoid_planning/frequency", frequency, 10);
    nh.param<float>("/obstacle_avoid_planning/xy_goal_tolerance", xy_goal_tolerance, 0.1);
    nh.param<float>("/obstacle_avoid_planning/yaw_goal_tolerance", yaw_goal_tolerance, 5);
    nh.param<int>("/obstacle_avoid_planning/follow_num", follow_num, 20);

    cout<< "max_velocity:" << max_velocity << endl;
    cout << "max_angular:" << max_angular << endl;
    cout << "frequency:" << frequency << endl;
    cout << "xy_goal_tolerance:" << xy_goal_tolerance << endl;
    cout << "yaw_goal_tolerance:" << yaw_goal_tolerance << endl;
    cout << "follow_num:" << follow_num << endl;

    pos_type = 0;
    plan_stage = None;
}

obstacle_avoid_planning ::~obstacle_avoid_planning()
{
    global_plan.clear();
}

bool obstacle_avoid_planning::planning_callback(yidamsg::auto_obstacle_avoid::Request &req, yidamsg::auto_obstacle_avoid::Response &res)
{
    ROS_INFO("start avoid from task_manager!");
    obs_distance = req.distance;
    isAvoid = true;
    res.result = true;
    return true;
}

void obstacle_avoid_planning::path_callback(const nav_msgs::PathConstPtr &path_msg)
{
    //cout << "get planing path " << endl;
    if (path_msg->poses.size() > 0 && plan_stage != None && plan_stage != Finish)
    {
        path = *path_msg;
        path_mutex.lock();
        global_plan.clear();
        for (int i = path_msg->poses.size(); i > 0; i--)
        {
            /* code */
            geometry_msgs::PoseStamped c_pose = path_msg->poses[i - 1];
            global_plan.push_back(c_pose);
        }
        current_num = 0;
        path_mutex.unlock();
    }
}

void obstacle_avoid_planning::update()
{
    //处理异步逻辑
    if (isAvoid)
    {
        //请求move_base
        bool isOk = client->isServerConnected();
        if (isOk)
        {
            //call service get new goal
            yidamsg::get_goal srv;
            srv.request.distance = obs_distance;
            if (planning_client.call(srv))
            {
                if (srv.response.success)
                {
                    //TODO:开启 平滑
                    std_srvs::SetBool ssrv;
                    ssrv.request.data = true;
                    if (smooth_client.call(ssrv))
                    {
                        ROS_INFO("open velocity smoother success!");
                        //set goal
                        goal.header.stamp = ros::Time::now();
                        goal.header.frame_id = "map";
                        goal.pose.position.x = srv.response.pos_x;
                        goal.pose.position.y = srv.response.pos_y;
                        goal.pose.position.z = srv.response.pos_z;
                        goal.pose.orientation.x = srv.response.qua_x;
                        goal.pose.orientation.y = srv.response.qua_y;
                        goal.pose.orientation.z = srv.response.qua_z;
                        goal.pose.orientation.w = srv.response.qua_w;

                        Quaternionf quanternion = Quaternionf(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z);
                        navigation_qua << goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

                        cout << "planning respose :";
                        cout << "pos_x:" << srv.response.pos_x;
                        cout << "pos_y:" << srv.response.pos_y;
                        cout << "pos_z:" << srv.response.pos_z;
                        cout << "qua_x:" << srv.response.qua_x;
                        cout << "qua_y:" << srv.response.qua_y;
                        cout << "qua_z:" << srv.response.qua_z;
                        cout << "qua_w:" << srv.response.qua_w << endl;
                        move_base_msgs::MoveBaseGoal base_goal;
                        base_goal.target_pose.header.stamp = ros::Time::now();
                        base_goal.target_pose.header.frame_id = "map";

                        base_goal.target_pose.pose.position.x = srv.response.pos_x;
                        base_goal.target_pose.pose.position.y = srv.response.pos_y;
                        base_goal.target_pose.pose.position.z = srv.response.pos_z;

                        base_goal.target_pose.pose.orientation.x = srv.response.qua_x;
                        base_goal.target_pose.pose.orientation.y = srv.response.qua_y;
                        base_goal.target_pose.pose.orientation.z = srv.response.qua_z;
                        base_goal.target_pose.pose.orientation.w = srv.response.qua_w;
                        client->sendGoal(base_goal, &obstacle_avoid_planning::doneCb, &obstacle_avoid_planning::activeCb, &obstacle_avoid_planning::feedbackCb);
                    }
                    else
                    {
                        ROS_ERROR("open velocity smoother failed!");
                        obstacle_avoid_planning::move_base_result = 2;
                    }
                }
                else
                {
                    obstacle_avoid_planning::move_base_result = 2;
                }
            }
            else
            {
                ROS_ERROR("failed to get new goal from obstacle_detection！");
                obstacle_avoid_planning::move_base_result = 2;
            }
        }
        else
        {
            obstacle_avoid_planning::move_base_result = 2;
        }

        isAvoid = false;
    }
    if (obstacle_avoid_planning::move_base_result > 0)
    {
        plan_stage = Finish;
        geometry_msgs::Twist motor_control;
        motor_control.linear.x = 0;
        motor_control.angular.z = 0;
        cmd_pub.publish(motor_control);

        yidamsg::obstacle_avoid_result srv;
        srv.request.flag = obstacle_avoid_planning::move_base_result == 1 ? 1 : 0;
        if (task_mananger_client.call(srv))
        {
            ROS_INFO("get task_mananger response");
        }
        else
        {
            ROS_ERROR("task_mananger response error");
        }
        obstacle_avoid_planning::move_base_result = 0;
        //TODO:关闭 平滑
        std_srvs::SetBool ssrv;
        ssrv.request.data = false;
        if (smooth_client.call(ssrv))
        {
            ROS_INFO("close velocity smoother success!");
        }
        else
        {
            ROS_ERROR("close velocity smoother failed!");
        }
    }
    //计算下一个目标点
    if (global_plan.size() < 1)
    {
        ROS_DEBUG("waiting for path ");
        return;
    }
    //cout << "have global plan " << endl;
    path_mutex.lock();
    int path_size = (follow_num < global_plan.size()) ? follow_num : global_plan.size();
    next_goal = Vector3d(0, 0, 0);
    next_goal[0] = path.poses[path_size - 1].pose.position.x;
    next_goal[1] = path.poses[path_size - 1].pose.position.y;
    next_goal[2] = path.poses[path_size - 1].pose.position.z;
    path_mutex.unlock();
    if (plan_stage == Await)
    {
        plan_stage = Plan;
    }
}

void obstacle_avoid_planning::control_callback(const std_msgs::Int16::ConstPtr &msg)
{
    //cout << " update goal " << endl;
    if (msg->data == 1)
    {
        sim_move(a_pose);
    }
    else if (msg->data == 2)
    {
        sim_move(b_pose);
    }
    else if (msg->data == 3)
    {
        sim_move(c_pose);
    }
    else if (msg->data == 4)
    {
        sim_move(d_pose);
    }
}

void obstacle_avoid_planning::sim_move(geometry_msgs::PoseStamped goal)
{
    Quaternionf quanternion = Quaternionf(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z);
    navigation_qua << goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

    move_base_msgs::MoveBaseGoal base_goal;
    base_goal.target_pose.header.stamp = ros::Time::now();
    base_goal.target_pose.header.frame_id = "map";

    base_goal.target_pose.pose.position.x = goal.pose.position.x;
    base_goal.target_pose.pose.position.y = goal.pose.position.y;
    base_goal.target_pose.pose.position.z = goal.pose.position.z;

    base_goal.target_pose.pose.orientation.x = goal.pose.orientation.x;
    base_goal.target_pose.pose.orientation.y = goal.pose.orientation.y;
    base_goal.target_pose.pose.orientation.z = goal.pose.orientation.z;
    base_goal.target_pose.pose.orientation.w = goal.pose.orientation.w;
    client->sendGoal(base_goal, &obstacle_avoid_planning::doneCb, &obstacle_avoid_planning::activeCb, &obstacle_avoid_planning::feedbackCb);
}

void obstacle_avoid_planning::deal_str(string topic, geometry_msgs::PoseStamped *goal_msg)
{
    goal_msg->header.frame_id = "map";
    goal_msg->header.stamp = ros::Time::now();
    boost::regex rgx("\\s+");
    boost::sregex_token_iterator iter(topic.begin(), topic.end(), rgx, -1);
    boost::sregex_token_iterator end;
    std::vector<std::string> topics;
    int index = 0;
    for (; iter != end; ++iter)
    {
        index++;
        double x = stringToNum<double>(*iter);
        //cout << "index:" << index << " value:" << x << endl;
        switch (index)
        {
        case 1:
            goal_msg->pose.position.x = x;
            continue;
        case 2:
            goal_msg->pose.position.y = x;
            continue;
        case 3:
            goal_msg->pose.position.z = x;
            continue;
        case 4:
            goal_msg->pose.orientation.x = x;
            continue;
        case 5:
            goal_msg->pose.orientation.y = x;
            continue;
        case 6:
            goal_msg->pose.orientation.z = x;
            continue;
        case 7:
            goal_msg->pose.orientation.w = x;
            continue;
        default:
            continue;
        }
        //ROS_INFO_STREAM("" << *iter);
    }
}

pair<double, double> obstacle_avoid_planning::calc_twist(float angle, float distance)
{
    //线速度应该与距离有关，当距离越近时线速度越小
    float coefficient = 0;
    if (angle > 30)
    {
        return make_pair(0, 2);
    }
    else if (angle <= 30 && angle > 5)
    {
        coefficient = distance > 0.3 ? 0.1 : 0.05;
        return make_pair(coefficient * max_velocity, 1);
    }
    coefficient = distance > 0.3 ? 0.3 : 0.1;
    return make_pair(coefficient * max_velocity, 0);
}

void obstacle_avoid_planning ::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    //cout << "get pose" << endl;
    //save data
    Quaternionf quanternion = Quaternionf(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    robot_qua << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

    robot_pose.pose.position.x = pose_msg->pose.pose.position.x;
    robot_pose.pose.position.y = pose_msg->pose.pose.position.y;
    robot_pose.pose.position.z = pose_msg->pose.pose.position.z;
    robot_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    robot_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    robot_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    robot_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
    geometry_msgs::Twist motor_control;
    if (plan_stage == Await)
    {
        ROS_INFO("waiting for plan");
        motor_control.linear.x = 0;
        motor_control.angular.z = 0;
        cmd_pub.publish(motor_control);
        return;
    }
    if (plan_stage == Finish)
    {
        ROS_INFO("navigation finish");
        motor_control.linear.x = 0;
        motor_control.angular.z = 0;
        cmd_pub.publish(motor_control);
        return;
    }
    publish_new_goal();
    //判断是否到达终点
    //calc cmd
    if (plan_stage == ArriveArea)
    {
        motor_control.linear.x = 0;
        double delta_angle = robot_qua[3] - navigation_qua[3];
        if (abs(delta_angle) > yaw_goal_tolerance)
        {
            //旋转时需要每次判断，不然容易出现原地打转的情况
            Eigen::Quaterniond q(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z);
            Eigen::Vector3d v(1, 0, 0);
            Eigen::Vector3d f = q * v;

            next_pose.pose.position.x = goal.pose.position.x + f[0] * 5;
            next_pose.pose.position.y = goal.pose.position.y + f[1] * 5;
            next_pose.pose.position.z = goal.pose.position.z + f[2] * 5;

            pair<double, double> dis_ang = calculate_distance_angle(robot_pose, next_pose);
            pair<double, double> twist = calc_twist(dis_ang.second, dis_ang.first);
            if (pos_type == 1 || pos_type == 3)
            {
                ROS_INFO("Reach the area and rotate right");
                motor_control.angular.z = twist.second;
            }
            else
            {
                ROS_INFO("Reach the area and rotate left");
                motor_control.angular.z = -twist.second;
            }
        }
        else
        {
            ROS_INFO("Arrive area and end navigation");
            //cancel
            client->cancelGoal();
            plan_stage == Finish;
            motor_control.angular.z = 0;

            obstacle_avoid_planning::move_base_result = 1;
        }
        cmd_pub.publish(motor_control);
    }
    else if (plan_stage == Plan)
    {
        //plan
        next_pose.pose.position.x = next_goal[0];
        next_pose.pose.position.y = next_goal[1];
        next_pose.pose.position.z = next_goal[2];
        pair<double, double> dis_ang = calculate_distance_angle(robot_pose, next_pose);
        if (!isArrived(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z))
        {
            if (pos_type == 1)
            {
                //forward right
                pair<double, double> twist = calc_twist(dis_ang.second, dis_ang.first);
                motor_control.linear.x = twist.first;
                motor_control.angular.z = twist.second;
            }
            else if (pos_type == 2)
            {
                //forward left
                pair<double, double> twist = calc_twist(dis_ang.second, dis_ang.first);
                motor_control.linear.x = twist.first;
                motor_control.angular.z = -twist.second;
            }
            else if (pos_type == 3)
            {
                //behind right
                pair<double, double> twist = calc_twist(dis_ang.second, dis_ang.first);
                motor_control.linear.x = 0;
                motor_control.angular.z = twist.second;
            }
            else if (pos_type == 4)
            {
                //behind left
                pair<double, double> twist = calc_twist(dis_ang.second, dis_ang.first);
                motor_control.linear.x = 0;
                motor_control.angular.z = -twist.second;
            }
            cmd_pub.publish(motor_control);
        }
        else
        {
            //cout << "flag_arriving_area " << endl;
            //calc rotate type
            plan_stage = ArriveArea;
        }
    }
}

double obstacle_avoid_planning::radian_to_angle(double radian)
{
    return 180 * radian / M_PI;
}

void obstacle_avoid_planning ::goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
{
    ROS_INFO("get new goal ");
    goal = *goal_msg;
    sim_move(goal);
}

pair<double, double> obstacle_avoid_planning::calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos)
{
    //忽略z方向
    self_pos.pose.position.z = goal_pos.pose.position.z;
    //calc robot goal relative position
    //dir = Vect(, goal_pos.pose.position.y - self_pos.pose.position.y, goal_pos.pose.position.z - self_pos.pose.position.z);
    float delta_x = goal_pos.pose.position.x - self_pos.pose.position.x;
    float delta_y = goal_pos.pose.position.y - self_pos.pose.position.y;
    float delta_z = goal_pos.pose.position.z - self_pos.pose.position.z;
    dir << delta_x, delta_y, delta_z;
    //calc forward direction
    Eigen::Quaterniond q(self_pos.pose.orientation.w, self_pos.pose.orientation.x, self_pos.pose.orientation.y, self_pos.pose.orientation.z);
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d f = q * v;
    float d1 = dot(f, normalized(dir));

    //calc right direction
    Eigen::AngleAxisd QX90(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond t_Q(QX90);
    Eigen::Vector3d r = q * t_Q * v;

    float d2 = dot(r, normalized(dir));
    if (d1 > 0)
    {
        if (d2 > 0)
        {
            pos_type = 1;
        }
        else
        {
            pos_type = 2;
        }
    }
    else
    {
        if (d2 > 0)
        {
            pos_type = 3;
        }
        else
        {
            pos_type = 4;
        }
    }
    //calc angle
    double angle = acos(d1) * 180 / 3.1415926;
    //calc distance
    double distance = sqrt(pow(goal_pos.pose.position.x - self_pos.pose.position.x, 2) + pow(goal_pos.pose.position.y - self_pos.pose.position.y, 2));

    ROS_DEBUG("distance: %.4f angle:%.4f", distance, angle);

    return make_pair(distance, angle);
}

float obstacle_avoid_planning::dot(Vector3d v1, Vector3d v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

Vector3d obstacle_avoid_planning::normalized(Vector3d v1)
{
    double mag = length(v1);
    Eigen::Vector3d v(v1[0] / mag, v1[1] / mag, v1[2] / mag);
    return v;
}

double obstacle_avoid_planning::length(Vector3d v1)
{
    return sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
}

void obstacle_avoid_planning::publish_new_goal()
{
    if (obstacle_avoid_planning::move_base_result != 0){
        nav_msgs::Odometry rodom;
        rodom.header.frame_id = "map";
        rodom.header.stamp = ros::Time::now();
        rodom.pose.pose.position.x = goal.pose.position.x;
        rodom.pose.pose.position.y = goal.pose.position.y;
        rodom.pose.pose.position.z = goal.pose.position.z;
        rodom.pose.pose.position.x = goal.pose.position.x;
        rodom.pose.pose.position.y = goal.pose.position.y;
        rodom.pose.pose.position.z = goal.pose.position.z;

        avoid_goal_pub.publish(rodom);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoid_planning");
    obstacle_avoid_planning pp;
    ROS_INFO("path_sim node started...");

    ros::Rate rate(10);
    while (ros::ok())
    {
        pp.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}