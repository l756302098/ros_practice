#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/once.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <velocity_smoother.h>
#include <tf/tf.h>
#include "std_msgs/Float32.h"
#include <pid.h>
//#include "yidamsg/wali_go_to_position.h"

#ifndef NAV_INTELLIGENT_PLAN_VALUES_H_
#define NAV_INTELLIGENT_PLAN_VALUES_H_

using namespace std;
using namespace Eigen;

enum PlanStage
{
    NONE,
    WAIT,
    ACTIVE,
    SUCCEEDED,
    CANCEL,
    FAILED,
};

typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class intelligent_plan
{
private:
    /* param */
    std::string _path_topic,_pose_topic,_cmd_topic;
	double _speed_kp,_speed_ki,_speed_kd,_angule_kp,_angule_ki,_angule_kd,_max_throttle,_goal_throttle;
 	PID pid_steer,pid_speed;
    bool is_cancel,_valid_path;
    int flag_path, flag_goal, flag_arriving_area, flag_arriving_direction;
    Vector4f robot_pose, navigation_goal;
    Vector3f next_goal;
    nav_msgs::Path path;
    int goal_seq;

    ros::Publisher planing_result_pub,cmd_pub,simple_goal_pub,cancle_pub;
    ros::Subscriber test_sub,pose_sub,path_sub,click_sub,goal_sub,cart_pose_sub;
    ros::ServiceServer task_service;
    void connect_server();
    void path_callback(const nav_msgs::PathConstPtr& path_msg);
    void pose_callback(const nav_msgs::OdometryConstPtr& pose_msg);
    void cart_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
	void click_callback(const geometry_msgs::PointStampedConstPtr& goal_msg);
    void goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
    bool isvalid_path(const nav_msgs::PathConstPtr& path_msg);
    bool isvalid_goal(const geometry_msgs::Pose& pose);
    void publishZero();

public:
    intelligent_plan(/* args */);
    ~intelligent_plan();
    double radian_to_angle(double radian)
    {
        return 180 * radian / M_PI;
    }
    void update();
    pair<double, double> pid_twist(pair<double, double> robot_goal_distance_angle,bool is_left);
    void test(const std_msgs::Bool::ConstPtr &msg);
    //bool task_service_cb(yidamsg::wali_go_to_position::Request &req,yidamsg::wali_go_to_position::Response &res);
    //communication with move_base
    Client *client;
    static PlanStage plan_stage;
    static void activeCb()
    {
        intelligent_plan::plan_stage = PlanStage::ACTIVE;
        ROS_INFO("Goal just went active");
    }
    static void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        ROS_INFO_STREAM_THROTTLE(1,"feedbackCb:" << feedback->base_position);
    }
    static void doneCb(const actionlib::SimpleClientGoalState &state,
                       const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        if (state == state.SUCCEEDED)
        {
            ROS_INFO("navigation success!");
            intelligent_plan::plan_stage = PlanStage::SUCCEEDED;
        }
        else if (state == state.PREEMPTED)
        {
            ROS_INFO("client cancel job.");
            intelligent_plan::plan_stage = PlanStage::CANCEL;
        }
        else
        {
            ROS_INFO("failed avoid obs");
            intelligent_plan::plan_stage = PlanStage::FAILED;
        }
    }
};

PlanStage intelligent_plan::plan_stage = PlanStage::NONE;

#endif
