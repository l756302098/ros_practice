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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "std_msgs/Int32.h"
#include <diagnostic_msgs/DiagnosticArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <yd_obstacle_avoid/intelligent_plan.h>
#include <yd_obstacle_avoid/obstacle_detection.h>

#include <yidamsg/ControlMode.h>

namespace yd_obstacle_avoid
{

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<float, 4, 1> Vector4f;

intelligent_plan::intelligent_plan(/* args */)
{
    calc_goal_err = 0;
    is_check = true;
    is_planning = false;
    connect_server();

    string localmap_topic,
        odom_topic, task_topic, raw_vel_topic, smooth_vel_topic, robot_odom_topic;
    nh.param<string>("/intelligent_plan_node/localmap_topic", localmap_topic, "/move_base/global_costmap/costmap");
    nh.param<string>("/intelligent_plan_node/odom_topic", odom_topic, "/odom_localization");
    nh.param<string>("/intelligent_plan_node/task_topic", task_topic, "/task_status");
    nh.param<std::string>("/intelligent_plan_node/raw_cmd_vel_topic", raw_vel_topic, "");
    nh.param<std::string>("/intelligent_plan_node/smooth_cmd_vel_topic", smooth_vel_topic, ""); //odom_topic
    nh.param<std::string>("/intelligent_plan_node/robot_odom_topic", robot_odom_topic, "");
    //nh.param<int>("/intelligent_plan_node/pc2laser_frequency", pc2laser_frequency_, 5);
    nh.param<int>("/intelligent_plan_node/smoother_frequency", smoother_frequency_, 10);
    cout << "localmap_topic:" << localmap_topic;
    cout << " odom_topic:" << odom_topic;
    cout << " task_topic:" << task_topic << endl;
    /**/
    //obstacle_detection
    ROS_INFO("obstacle detection init");
    //sub
    robot_pose_sub = nh.subscribe(odom_topic, 1, &obstacle_detection_new::pose_callback, &od);
    map_sub = nh.subscribe(localmap_topic, 1, &obstacle_detection_new::set_map, &od);
    task_sub = nh.subscribe(task_topic, 1, &obstacle_detection_new::task_status, &od);
    new_goal_sub = nh.subscribe("/yida/obstacle_avoid/new_goal", 1, &obstacle_detection_new::new_goal, &od);
    control_model_sub = nh.subscribe("/control_mode", 1, &intelligent_plan::interrupt, this);
    test_sub = nh.subscribe("/yida/obstacle_avoid/test", 1, &intelligent_plan::test, this);
    //pub
    control_model_pub = nh.advertise<yidamsg::ControlMode>("/yida/robot/control_mode", 1, true);
    result_pub = nh.advertise<std_msgs::Int32>("/yida/obstacle_avoid/result", 1, true);
    obstacle_pub = nh.advertise<std_msgs::Float32>("/yida/obstacle_avoid/distance", 1, true);
    hearbeat_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/yd/heartbeat", 1, true);
    //time tick
    hb_timer = nh.createTimer(ros::Duration(1.0), &intelligent_plan::hearbeat, this, false);
    //other
    ROS_INFO("smoother_thread_ init");
    //smoother_thread_ = new boost::thread(boost::bind(&intelligent_plan::smoother_thread, this));
    ROS_INFO("pc2laser_thread_ init");
    pc2laser_thread_ = new boost::thread(boost::bind(&intelligent_plan::pc2laser_thread, this));
}

intelligent_plan::~intelligent_plan()
{
    smoother_thread_ = NULL;
    pc2laser_thread_ = NULL;
}

void intelligent_plan::smoother_thread()
{
    vs = new velocity_smoother();
    ros::Rate rate(smoother_frequency_);
    int flag = 0;
    while (ros::ok())
    {
        if (intelligent_plan::is_smooth)
        {
            if (flag != 0){
                flag = 0;
                vs->reset();
            }
            vs->update();
        }else{
            if (flag == 0)
            {
                flag++;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void intelligent_plan::pc2laser_thread()
{
    ps = new pc2ls();
}

void intelligent_plan::control_mode(int model)
{
    // 0 is task model
    // 1 is joy model
    // 4 is obstacle model
   if(model == 4){
       is_planning = true;
   }else{
       is_planning = false;
   }
    yidamsg::ControlMode msg;
    msg.robot_id = 1;
    msg.mode = model;
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
    obs_dis = 0;
    bool is_find = od.detection(obs_dis);
    pub_distance(obs_dis);
    /*
    if (intelligent_plan::move_base_result != 0)
    {
        float sleep_time = intelligent_plan::move_base_result == 1 ? 10 : 30;
        ROS_INFO_STREAM("move_base_result:" << intelligent_plan::move_base_result);
        ROS_INFO_STREAM("Task execution finished! sleep:" << sleep_time << " now:" << ros::Time::now());
        pub_hearbeat(0, "Task execution finished!");
        is_check = false;
        timer = nh.createTimer(ros::Duration(sleep_time), &intelligent_plan::recover, this, true);
        if (intelligent_plan::move_base_result == 1 || intelligent_plan::move_base_result == 3 ||
            intelligent_plan::move_base_result == 5)
        {
            //reset
            intelligent_plan::is_smooth = false;
            intelligent_plan::plan_stage = PlanStage::NONE;
            //change control model
            ROS_INFO("change control model:0");
            pub_hearbeat(0, "change control model:0");
            control_mode(0);
            sleep(2.0);
            //notify task manager
            pub_result(intelligent_plan::move_base_result);
            intelligent_plan::move_base_result = 0; 
        }
        else if (intelligent_plan::move_base_result==2)
        {
            intelligent_plan::is_smooth = false;
            intelligent_plan::plan_stage = PlanStage::NONE;
            //失败后回到起点
            failed_recover();
        }
        else if (intelligent_plan::move_base_result == 4 || intelligent_plan::move_base_result == 6 ||
                 intelligent_plan::move_base_result == 10)
        {
            //reset
            intelligent_plan::is_smooth = false;
            intelligent_plan::plan_stage = PlanStage::NONE;
            //change control model
            ROS_INFO("change control model:0");
            pub_hearbeat(0, "change control model:0");
            control_mode(0);
            sleep(2.0);
            pub_result(intelligent_plan::move_base_result);
            intelligent_plan::move_base_result = 0;
        }
    }
    if (intelligent_plan::plan_stage != PlanStage::NONE)
        return;
    //detection obatacle
    obs_dis = 0;
    if (!is_check){
        pub_distance(obs_dis);
        return;
    }  
    if (od.detection(obs_dis))
    {
        
        geometry_msgs::Pose target_pos;
        int result = od.calc_new_goal(obs_dis, target_pos);
        ROS_INFO_STREAM("calc goal result:" << result);
        
        intelligent_plan::plan_stage = PlanStage::WAIT;
        bool isOk = client->isServerConnected();
        if (isOk)
        {
            //change control model
            pub_hearbeat(0, "change control model:4");
            control_mode(4);
            //calc new goal
            new_planning(obs_dis);
        }
        else
        {
            pub_hearbeat(2, "Action server is disconnect!");
            intelligent_plan::plan_stage = PlanStage::NONE;
        }
        
    }else{
        if (calc_goal_err!=0){
            //reset control_mode
            ROS_INFO("Obstacles disappear, reset control_mode 0");
            pub_hearbeat(0, "Obstacles disappear, reset control_mode 0");
            control_mode(0);
            calc_goal_err = 0;
        }
    }
    pub_distance(obs_dis);
    */
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

void intelligent_plan::hearbeat(const ros::TimerEvent &event)
{
    //ROS_INFO("hearbeat ...");
    pub_hearbeat(0);
}

void intelligent_plan::recover(const ros::TimerEvent &event)
{
    ROS_INFO("is check recover");
    ROS_INFO_STREAM("recover time:" << ros::Time::now());
    is_check = true;
}

void intelligent_plan::test(const std_msgs::Float32::ConstPtr &msg)
{
    int flag = msg->data;
    if(flag==1){
        intelligent_plan::is_smooth = true;
        control_mode(4);
    }else if (flag ==0)
    {
        intelligent_plan::is_smooth = false;
        control_mode(0);
    }
    
}

void intelligent_plan::new_planning(float dis_)
{
    geometry_msgs::Pose new_goal;
    int result = od.calc_new_goal(dis_, new_goal);
    if (result==1)
    {
        calc_goal_err = 0;
        //open smooth
        ROS_INFO("start smooth ");
        intelligent_plan::is_smooth = true;
        ROS_INFO_STREAM("new goal:" << new_goal);
        pub_hearbeat(0, "start intelligent move!");
        //start auto move
        move_base_msgs::MoveBaseGoal base_goal;
        base_goal.target_pose.header.stamp = ros::Time::now();
        base_goal.target_pose.header.frame_id = "map";
        base_goal.target_pose.pose = new_goal;
        client->sendGoal(base_goal, &intelligent_plan::doneCb, &intelligent_plan::activeCb, &intelligent_plan::feedbackCb);
    }
    else if (result == 0)
    {
        calc_goal_err++;
        ROS_ERROR("calc new goal failed! %i", calc_goal_err);
        //ROS_ERROR("notify task manager! end waitting");
        pub_hearbeat(1, "calc new goal failed!");
        intelligent_plan::plan_stage = PlanStage::NONE;

        //等待一段时间,再次进行判断
        if (calc_goal_err < 10)
        {
            is_check = false;
            ROS_INFO_STREAM("now:"<<ros::Time::now());
            timer = nh.createTimer(ros::Duration(1.0), &intelligent_plan::recover, this, true);
        }
        else
        {
            calc_goal_err = 0;
            // failed
            move_base_result = 2;
        }
    }
    else if (result == 2){
        //路宽不够
        pub_result(2);
    }
}
void intelligent_plan::interrupt(const yidamsg::ControlMode::ConstPtr &msg)
{
    if (is_planning && msg->mode != 4)
    {
        //cancel goal
        ROS_INFO("interrupt yd_obstacle_avoid task");
        pub_hearbeat(0, "interrupt yd_obstacle_avoid task");
        bool isOk = client->isServerConnected();
        if (isOk)
        {
            client->cancelAllGoals();
        }
        is_planning = false;
        intelligent_plan::move_base_result = 10;
    }
    else if (!is_planning && msg->mode == 4)
    {
        if (!od.is_had_pos)
            return;
        if (!od.is_add_map)
            return;
        if (od.start_x==0 && od.start_y==0)
            return;       
        //获取控制权
        control_mode(4);
        intelligent_plan::move_base_result = 4;
        //recover
        failed_recover();
    }
}

void intelligent_plan::pub_result(int result)
{
    std_msgs::Int32 msg;
    msg.data = result;
    result_pub.publish(msg);
}

void intelligent_plan::failed_recover()
{
    calc_goal_err = 0;
    //找到起点
    float p1x = od.end_x - od.start_x;
    float p1y = od.end_y - od.start_y;
    float p2x = 1;
    float p2y = 0;
    //get p0 p1 p2 angle
    float theta = atan2(p1x, p1y) - atan2(p2x, p2y);
    if (theta > M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;
    float angle = theta * 180.0 / M_PI;
    geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(theta);
    //open smooth
    ROS_INFO("start smooth ");
    intelligent_plan::is_smooth = true;
    pub_hearbeat(0, "start intelligent move!");
    //start auto move
    move_base_msgs::MoveBaseGoal base_goal;
    base_goal.target_pose.header.stamp = ros::Time::now();
    base_goal.target_pose.header.frame_id = "map";
    base_goal.target_pose.pose.position.x = od.start_x;
    base_goal.target_pose.pose.position.y = od.start_y;
    base_goal.target_pose.pose.position.z = 0;

    base_goal.target_pose.pose.orientation.x = qua.x;
    base_goal.target_pose.pose.orientation.y = qua.y;
    base_goal.target_pose.pose.orientation.z = qua.z;
    base_goal.target_pose.pose.orientation.w = qua.w;
    client->sendGoal(base_goal, &intelligent_plan::doneCb, &intelligent_plan::activeCb, &intelligent_plan::feedbackCb);
}

void intelligent_plan::pub_distance(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    obstacle_pub.publish(msg);
}
}