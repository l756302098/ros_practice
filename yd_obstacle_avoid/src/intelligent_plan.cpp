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
#include "std_msgs/Int32.h"
#include <diagnostic_msgs/DiagnosticArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <yd_obstacle_avoid/intelligent_plan.h>
#include <yd_obstacle_avoid/obstacle_detection.h>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<float, 4, 1> Vector4f;

intelligent_plan::intelligent_plan(/* args */)
{
    //connect_server();

    string localmap_topic, odom_topic, task_topic;
    nh.param<string>("/intelligent_plan_node/localmap_topic", localmap_topic, "/move_base/global_costmap/costmap");
    nh.param<string>("/intelligent_plan_node/odom_topic", odom_topic, "/odom_localization");
    nh.param<string>("/intelligent_plan_node/task_topic", task_topic, "/task_status");

    nh.param<int>("/intelligent_plan_node/smoother_frequency", smoother_frequency_, 10);
    nh.param<int>("/intelligent_plan_node/pc2laser_frequency", pc2laser_frequency_, 5);
    cout << "localmap_topic:" << localmap_topic;
    cout << " odom_topic:" << odom_topic;
    cout << " task_topic:" << task_topic << endl;
    /**/
    //obstacle_detection
    ROS_INFO("obstacle detection init");
    robot_pose_sub = nh.subscribe(odom_topic, 1, &obstacle_detection::pose_callback, &od);
    map_sub = nh.subscribe(localmap_topic, 1, &obstacle_detection::set_map, &od);
    task_sub = nh.subscribe(task_topic, 1, &obstacle_detection::task_status, &od);
    ROS_INFO("other init");
    //other
    control_model_pub = nh.advertise<std_msgs::Int32>("/yida/robot/control_mode", 1, true);
    obstacle_pub = nh.advertise<std_msgs::Int32>("/yida/obstacle_avoid/result", 1, true);
    hearbeat_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/yida/hearbeat", 1, true);
    
    ROS_INFO("smoother_thread_ init");
    smoother_thread_ = new boost::thread(boost::bind(&intelligent_plan::smoother_thread, this));
    ROS_INFO("pc2laser_thread_ init");
    pc2laser_thread_ = new boost::thread(boost::bind(&intelligent_plan::pc2laser_thread, this));
}

intelligent_plan::~intelligent_plan()
{
}

void intelligent_plan::smoother_thread()
{
    if (smoother_frequency_ <= 0)
        return;
    //calc usleep(1000);
    int sleep_time = 1000 / smoother_frequency_;
    while (ros::ok())
    {
        //ROS_INFO("smoother update");
        vs.update();
        usleep(sleep_time);
    }
}

void intelligent_plan::pc2laser_thread()
{
    if (pc2laser_frequency_ <= 0)
        return;
    //calc usleep(1000);
    int sleep_time = 1000 / pc2laser_frequency_;
    while (ros::ok())
    {
        //ROS_INFO("smoother update");
        ps.deal_queue();
        usleep(sleep_time);
    }
}

void intelligent_plan::control_mode(int model)
{
    // 0 is task model
    // 4 is obstacle model
    std_msgs::Int32 msg;
    msg.data = model;
    control_model_pub.publish(msg);
}

void intelligent_plan::connect_server()
{
    client = new Client("move_base", true);
    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

void intelligent_plan::update()
{
    pub_hearbeat(0);
    if (intelligent_plan::move_base_result != 0)
    {
        //change control model
        ROS_INFO("change control model:0");
        std_msgs::Int32 msg;
        msg.data = 0;
        control_model_pub.publish(msg);
        //notify task manager
        std_msgs::Int32 msg2;
        msg2.data = intelligent_plan::move_base_result;
        obstacle_pub.publish(msg2);
        intelligent_plan::move_base_result = 0;
        //reset
        intelligent_plan::plan_stage = PlanStage::NONE;
    }
    if (intelligent_plan::plan_stage != PlanStage::NONE)
        return;
    //detection obatacle
    obs_dis = 0;
    if (od.detection(obs_dis))
    {
        intelligent_plan::plan_stage = PlanStage::WAIT;
        bool isOk = client->isServerConnected();
        if (isOk)
        {
            //change control model
            control_mode(4);
            //calc new goal
            geometry_msgs::PoseStamped new_goal;
            if (od.calc_new_goal(obs_dis, new_goal))
            {
                //start auto move
                move_base_msgs::MoveBaseGoal base_goal;
                base_goal.target_pose.header.stamp = ros::Time::now();
                base_goal.target_pose.header.frame_id = "map";
                base_goal.target_pose.pose.position.x = new_goal.pose.position.x;
                base_goal.target_pose.pose.position.y = new_goal.pose.position.y;
                base_goal.target_pose.pose.position.z = new_goal.pose.position.z;

                base_goal.target_pose.pose.orientation.x = new_goal.pose.orientation.x;
                base_goal.target_pose.pose.orientation.y = new_goal.pose.orientation.y;
                base_goal.target_pose.pose.orientation.z = new_goal.pose.orientation.z;
                base_goal.target_pose.pose.orientation.w = new_goal.pose.orientation.w;
                client->sendGoal(base_goal, &intelligent_plan::doneCb, &intelligent_plan::activeCb, &intelligent_plan::feedbackCb);
            }
            else
            {
                ROS_ERROR("calc new goal failed!");
                pub_hearbeat(1, "calc new goal failed!");
                intelligent_plan::plan_stage = PlanStage::NONE;
            }
        }
        else
        {
            pub_hearbeat(2, "Action server is disconnect!");
            intelligent_plan::plan_stage = PlanStage::NONE;
        }
    }
}

void intelligent_plan::pub_hearbeat(int level, string message)
{
    diagnostic_msgs::DiagnosticArray log;
    log.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus s;
    s.name = "/yd_obstacle_avoid"; // 这里写节点的名字
    s.level = level;               // 0 = OK, 1 = Warn, 2 = Error
    if (!message.empty())
    {
        s.message = message; // 问题描述
    }
    log.status.push_back(s);

    hearbeat_pub.publish(log);
}
