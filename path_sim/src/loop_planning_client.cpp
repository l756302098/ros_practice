#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <stdio.h>

#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <math.h>

#include "utils.cpp"

#include <yidamsg/wali_do_patrol.h>
#include <yidamsg/wali_task_status.h>
#include "std_msgs/Bool.h"

using namespace std;

enum LoopStage
{
    None,
    Wait,
    Loop,
    Finish,
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

std::string a_s, b_s, c_s, d_s;
geometry_msgs::PoseStamped a_pose, b_pose, c_pose, d_pose, current_pose;
int current_index;
LoopStage loop_status;
ros::Subscriber goal_sub_;
ros::Publisher cancel_pub;
Client *client;
ros::ServiceServer wali_do_srv_;
ros::ServiceClient wali_task_client_;
//temp var
move_base_msgs::MoveBaseGoal goal;
int command;

void cancel()
{
    cout << "client:cancel task" << endl;
    std_msgs::Bool status;
    status.data = true;
    cancel_pub.publish(status);
}

void callService(int status = -1)
{
    yidamsg::wali_task_status msg;
    msg.request.status_code = status;
    bool result = wali_task_client_.call(msg);
    std::cout << "result" << result << std::endl;
}

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState &state,
            const move_base_msgs::MoveBaseResultConstPtr &result)
{
    ROS_INFO("Yay! The dishes are now clean");
    //cout << state << endl;
    loop_status = Finish;
    // current_index++;
    // if (current_index >= 2)
    // {
    //     current_index = 0;
    // }
    if (state == state.ABORTED)
    {
        callService(-1);
    }
    else if (state == state.PREEMPTED)
    {
        ROS_INFO("client cancel job.");
    }
    else if (state == state.SUCCEEDED)
    {
        callService(1);
    }
    //判断终点
    bool isActive = client->isServerConnected();
    std::cout << "isActive" << isActive << std::endl;
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用的回调函数
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    //ROS_INFO(" feedbackCb : %f ", feedback->base_position);
}

void deal_str(string topic, geometry_msgs::PoseStamped *goal_msg)
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

void sendGoal(Client *client, int index)
{
    if (index == 0)
    {
        current_pose = a_pose;
    }
    else
    {
        current_pose = b_pose;
    }
    // 创建一个action的goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";

    goal.target_pose.pose.position.x = current_pose.pose.position.x;
    goal.target_pose.pose.position.y = current_pose.pose.position.y;
    goal.target_pose.pose.position.z = current_pose.pose.position.z;

    goal.target_pose.pose.orientation.x = current_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = current_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = current_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = current_pose.pose.orientation.w;

    // 发送action的goal给服务器端，并且设置回调函数
    client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
}

void sendGoal(Client *client, double p_x, double p_y, double p_z, double q_x, double q_y, double q_z, double q_w)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";

    goal.target_pose.pose.position.x = p_x;
    goal.target_pose.pose.position.y = p_y;
    goal.target_pose.pose.position.z = p_z;

    goal.target_pose.pose.orientation.x = q_x;
    goal.target_pose.pose.orientation.y = q_y;
    goal.target_pose.pose.orientation.z = q_z;
    goal.target_pose.pose.orientation.w = q_w;

    // 发送action的goal给服务器端，并且设置回调函数
    client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
}

void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal_)
{
    ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";

    goal.target_pose.pose.position.x = goal_->pose.position.x;
    goal.target_pose.pose.position.y = goal_->pose.position.y;
    goal.target_pose.pose.position.z = goal_->pose.position.z;

    goal.target_pose.pose.orientation.x = goal_->pose.orientation.x;
    goal.target_pose.pose.orientation.y = goal_->pose.orientation.y;
    goal.target_pose.pose.orientation.z = goal_->pose.orientation.z;
    goal.target_pose.pose.orientation.w = goal_->pose.orientation.w;

    // 发送action的goal给服务器端，并且设置回调函数
    client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
}

bool do_patrol(yidamsg::wali_do_patrol::Request &req,
               yidamsg::wali_do_patrol::Response &res)
{
    std::cout << "do_patrol ..." << std::endl;
    std::cout << req.patrol_command_code << std::endl;
    //int command = req.patrol_command_code;
    if (req.patrol_command_code == 1)
    {
        command = 1;

        std::cout
            << "pos x:" << req.pos_x << " y:" << req.pos_y << " z:" << req.pos_z << std::endl;
        std::cout
            << "qua x:" << req.ori_x << " y:" << req.ori_y << " z:" << req.ori_z << " w:" << req.ori_w << std::endl;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";

        goal.target_pose.pose.position.x = req.pos_x;
        goal.target_pose.pose.position.y = req.pos_y;
        goal.target_pose.pose.position.z = req.pos_z;

        goal.target_pose.pose.orientation.x = req.ori_x;
        goal.target_pose.pose.orientation.y = req.ori_y;
        goal.target_pose.pose.orientation.z = req.ori_z;
        goal.target_pose.pose.orientation.w = req.ori_w;
        command = 1;
    }
    else if (req.patrol_command_code == 2)
    {
        command = 2;
    }

    res.success = true;
    return true;
}

void update()
{
    if (command == 1)
    {
        client->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        command = 0;
    }
    else if (command == 2)
    {
        cancel();
        command = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_planning_node");
    ros::NodeHandle nh;

    command = 0;

    nh.param<std::string>("/loop_planning_client/a", a_s, "dd");
    nh.param<std::string>("/loop_planning_client/b", b_s, "dd");
    nh.param<std::string>("/loop_planning_client/c", c_s, "dd");
    nh.param<std::string>("/loop_planning_client/d", d_s, "dd");
    cout << "a_point:" << a_s << endl;
    cout << "b_point:" << b_s << endl;
    cout << "c_point:" << c_s << endl;
    cout << "d_point:" << d_s << endl;
    deal_str(a_s, &a_pose);
    cout << "a_pose:" << a_pose << endl;
    deal_str(b_s, &b_pose);
    cout << "b_pose:" << b_pose << endl;
    deal_str(c_s, &c_pose);
    cout << "c_pose:" << c_pose << endl;
    deal_str(d_s, &d_pose);
    cout << "d_pose:" << d_pose << endl;
    loop_status = Wait;

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, goalCB);
    cancel_pub = nh.advertise<std_msgs::Bool>("/loop_path/cancel", 1);
    //add service
    wali_do_srv_ = nh.advertiseService("/do_patrol", &do_patrol);
    wali_task_client_ = nh.serviceClient<yidamsg::wali_task_status>("do_patrol_task_status");
    // 定义一个客户端
    client = new Client("loop_planning", true);

    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started, sending goal.");

    ros::Rate rate(10);
    while (ros::ok())
    {
        //1.判断是否开始结束
        // if (loop_status != Loop)
        // {
        //     //sleep
        //     sleep(30);
        //     //next loop
        //     sendGoal(client, current_index);
        //     loop_status = Loop;
        // }
        update();
        ros::spinOnce();
        rate.sleep();
    }

    delete client;
    return 0;
}
