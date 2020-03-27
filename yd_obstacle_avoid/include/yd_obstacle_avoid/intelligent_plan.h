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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "yidamsg/ControlMode.h"
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <yd_obstacle_avoid/obstacle_detection.h>
#include <yd_obstacle_avoid/obstacle_detection_new.h>
#include <yd_obstacle_avoid/velocity_smoother.h>
#include <yd_obstacle_avoid/pc2ls.h>

#ifndef INTELLIGENT_PLAN_VALUES_H_
#define INTELLIGENT_PLAN_VALUES_H_
    namespace yd_obstacle_avoid
{

using namespace std;

enum PlanStage
{
    NONE,
    WAIT,
    ACTIVE,
};

typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class intelligent_plan
{
private:
    /* param */
    ros::Publisher control_model_pub, obstacle_pub, hearbeat_pub, pose_stop_pub;
    ros::Subscriber robot_pose_sub, map_sub, task_sub, new_goal_sub, test_sub, control_model_sub;
    obstacle_detection_new od;
    pc2ls *ps;
    velocity_smoother *vs;
    float obs_dis;
    void control_mode(int model);
    void connect_server();
    ros::NodeHandle nh;
    boost::thread *smoother_thread_, *pc2laser_thread_;
    int smoother_frequency_, pc2laser_frequency_;
    int calc_goal_err;
    bool is_check,is_planning;
    ros::Timer timer, hb_timer;

public:
    intelligent_plan(/* args */);
    ~intelligent_plan();
    void update();
    void pub_hearbeat(int level, string message = "");
    void hearbeat(const ros::TimerEvent &event);
    void recover(const ros::TimerEvent &event);
    void new_planning(float dis_);
    void test(const std_msgs::Float32::ConstPtr &msg);
    void interrupt(const yidamsg::ControlMode::ConstPtr & msg);
    //thread
    void smoother_thread();
    void pc2laser_thread();
    //communication with move_base
    Client *client;
    static PlanStage plan_stage;
    static int move_base_result;
    static bool is_obstacle;
    static bool is_smooth;
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
            ROS_INFO("success avoid obs");
            intelligent_plan::move_base_result = 1;
        }
        else if (state == state.PREEMPTED)
        {
            ROS_INFO("client cancel job.");
        }
        else
        {
            ROS_INFO("failed avoid obs");
            intelligent_plan::move_base_result = 2;
        }
    }
};

int intelligent_plan::move_base_result = 0;
bool intelligent_plan::is_smooth = false;
PlanStage intelligent_plan::plan_stage = PlanStage::NONE;
}
#endif