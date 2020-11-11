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
#include <intelligent_plan.h>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<float, 4, 1> Vector4f;

intelligent_plan::intelligent_plan(/* args */)
{
    ros::NodeHandle nh("~");
    nh.param("path_topic", _path_topic,	std::string("/move_base/GlobalPlanner/plan"));
	nh.param("pose_topic", _pose_topic,	std::string("/odom_localization"));
	nh.param("cmd_topic", _cmd_topic,	std::string("/mobile_base/commands/velocity"));
	nh.param("angle_kp", _angule_kp, 0.1);
	nh.param("angle_ki", _angule_ki, 0.001);
	nh.param("angle_kd", _angule_kd, 2.8);
	nh.param("speed_kp", _speed_kp, 0.0);
	nh.param("speed_ki", _speed_ki, 0.0);
	nh.param("speed_kd", _speed_kd, 0.0);
	nh.param("max_throttle", _max_throttle, 0.5);
	std::cout << "path_topic:" << _path_topic << " pose_topic:"<<_pose_topic  << std::endl;
	std::cout << "angle_kp:" << _angule_kp << " angle_ki:" << _angule_ki << " angle_kd:"<< _angule_kd << std::endl;
	std::cout << "speed_kp:" << _speed_kp << " speed_ki:" << _speed_ki << " speed_kd:"<< _speed_kd << " max_throttle："<<_max_throttle << std::endl;
 	ROS_INFO("intelligent_plan init");
    connect_server();
    flag_goal = 0;
	flag_path = 0;
	flag_arriving_area = 0;
	flag_arriving_direction = 0;
	pid_steer.Init(_angule_kp, _angule_ki, _angule_kd);
	pid_speed.Init(_speed_kp, _speed_ki, _speed_kd);
    //sub
    test_sub = nh.subscribe("/yida/obstacle_avoid/test", 1, &intelligent_plan::test, this);
    //server
    task_service = nh.advertiseService("/go_to_position", &intelligent_plan::task_service_cb,this);
    //move_base
	path_sub = nh.subscribe(_path_topic, 1, &intelligent_plan::path_callback, this);
	pose_sub = nh.subscribe(_pose_topic, 1, &intelligent_plan::pose_callback, this);
	cart_pose_sub = nh.subscribe("/cart/pose", 1, &intelligent_plan::cart_pose_callback, this);
	click_sub = nh.subscribe("/clicked_point", 1, &intelligent_plan::click_callback, this);
	cmd_pub = nh.advertise<geometry_msgs::Twist>(_cmd_topic, 5, true);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &intelligent_plan::goal_callback, this);
}

intelligent_plan::~intelligent_plan()
{
    
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
    if(intelligent_plan::plan_stage == PlanStage::NONE) return;
    if(intelligent_plan::plan_stage == PlanStage::ACTIVE){
        //check data
        
        //
    }
}

void intelligent_plan::test(const std_msgs::Float32::ConstPtr &msg)
{
    
}

bool intelligent_plan::task_service_cb(yidamsg::wali_go_to_position::Request &req,yidamsg::wali_go_to_position::Response &res){
    //开始task
    flag_goal = 1;
    flag_arriving_area = 0;
    flag_arriving_direction = 0;
    geometry_msgs::Pose new_goal;
    new_goal.position.x = req.pose.position.x;
    new_goal.position.y = req.pose.position.y;
    new_goal.position.z = req.pose.position.z;
    new_goal.orientation.x = req.pose.orientation.x;
    new_goal.orientation.y = req.pose.orientation.y;
    new_goal.orientation.z = req.pose.orientation.z;
    new_goal.orientation.w = req.pose.orientation.w;
    move_base_msgs::MoveBaseGoal base_goal;
    base_goal.target_pose.header.stamp = ros::Time::now();
    base_goal.target_pose.header.frame_id = "map";
    base_goal.target_pose.pose = new_goal;
    client->sendGoal(base_goal, &intelligent_plan::doneCb, &intelligent_plan::activeCb, &intelligent_plan::feedbackCb);
    return true;
}

void intelligent_plan::path_callback(const nav_msgs::PathConstPtr& path_msg){
    if(path_msg->poses.size()>0 && flag_goal)
    {
	    path = *path_msg;
	    int path_size = (100<path_msg->poses.size())?100:path_msg->poses.size();
	    next_goal = Vector3f(0, 0, 0);
	    for(int i=0; i<path_size; i++)
	    {
	        next_goal[0] += path.poses[i].pose.position.x;
	        next_goal[1] += path.poses[i].pose.position.y;
	        next_goal[2] += path.poses[i].pose.position.z;
	    }
	    next_goal = next_goal / path_size;
	    flag_path = 20;
    }
}

void intelligent_plan::cart_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg){
	Quaternionf quanternion = Quaternionf(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    robot_pose << pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0,1,2)[2]);
    geometry_msgs::PoseStamped pose;
	pose.pose.position.x = pose_msg->pose.position.x;
    pose.pose.position.y = pose_msg->pose.position.y;
    pose.pose.position.z = pose_msg->pose.position.z;
    pose.pose.orientation.x = pose_msg->pose.orientation.x;
    pose.pose.orientation.y = pose_msg->pose.orientation.y;
    pose.pose.orientation.z = pose_msg->pose.orientation.z;
    pose.pose.orientation.w = pose_msg->pose.orientation.w;
	if(flag_path > 0)
    {
        flag_path--;
	    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        //forward
        Eigen::Vector3d v(1, 0, 0);
        Eigen::Vector3d f = q * v;
        //rotate
        Eigen::AngleAxisd QX90(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond t_Q(QX90);
        Eigen::Vector3d r = q * t_Q * v;
	    float dx = next_goal[0] - pose.pose.position.x;
	    float dy = next_goal[1] - pose.pose.position.y;
	    float dotf = dx * f[0] + dy * f[1];
	    float dotr = dx * r[0] + dy * r[1];
	    double angle = radian_to_angle(acos(dotf/sqrt(dx * dx + dy * dy)));
	    double distance = sqrt(pow(navigation_goal[0]-robot_pose[0], 2) + pow(navigation_goal[1]-robot_pose[1], 2));
	    pair<double, double> twist = make_pair(0, 0);
	    if(dotr>0){
		    twist = pid_twist(make_pair(distance, angle),false);
	    }else{
		    twist = pid_twist(make_pair(distance, angle),true);
	    }	
	    geometry_msgs::Twist motor_control;
	    motor_control.linear.x = twist.first;
	    motor_control.angular.z = twist.second;
	    cmd_pub.publish(motor_control);
	    cout << "published v and w is: " << twist.first << "  " << twist.second << endl;
    }else{
		geometry_msgs::Twist motor_control;
		motor_control.linear.x = 0;
		motor_control.angular.z = 0;
		cmd_pub.publish(motor_control);
	}
}

void intelligent_plan::pose_callback(const nav_msgs::OdometryConstPtr& pose_msg){
    Quaternionf quanternion = Quaternionf(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    robot_pose << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0,1,2)[2]);
    geometry_msgs::PoseStamped pose;
	pose.pose.position.x = pose_msg->pose.pose.position.x;
    pose.pose.position.y = pose_msg->pose.pose.position.y;
    pose.pose.position.z = pose_msg->pose.pose.position.z;
    pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
	if(flag_path > 0)
    {
        flag_path--;
	    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        //forward
        Eigen::Vector3d v(1, 0, 0);
        Eigen::Vector3d f = q * v;
        //rotate
        Eigen::AngleAxisd QX90(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond t_Q(QX90);
        Eigen::Vector3d r = q * t_Q * v;
	    float dx = next_goal[0] - pose.pose.position.x;
	    float dy = next_goal[1] - pose.pose.position.y;
	    float dotf = dx * f[0] + dy * f[1];
	    float dotr = dx * r[0] + dy * r[1];
	    double angle = radian_to_angle(acos(dotf/sqrt(dx * dx + dy * dy)));
	    double distance = sqrt(pow(navigation_goal[0]-robot_pose[0], 2) + pow(navigation_goal[1]-robot_pose[1], 2));
	    pair<double, double> twist = make_pair(0, 0);
	    if(dotr>0){
		    twist = pid_twist(make_pair(distance, angle),false);
	    }else{
		    twist = pid_twist(make_pair(distance, angle),true);
	    }	
	    geometry_msgs::Twist motor_control;
	    motor_control.linear.x = twist.first;
	    motor_control.angular.z = twist.second;
	    cmd_pub.publish(motor_control);
	    cout << "published v and w is: " << twist.first << "  " << twist.second << endl;
    }else{
		geometry_msgs::Twist motor_control;
		motor_control.linear.x = 0;
		motor_control.angular.z = 0;
		cmd_pub.publish(motor_control);
	}
}
void intelligent_plan::click_callback(const geometry_msgs::PointStampedConstPtr& goal_msg){

}

void intelligent_plan::goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg){
    flag_goal = 1;
    flag_arriving_area = 0;
    flag_arriving_direction = 0;
    Quaternionf quanternion = Quaternionf(goal_msg->pose.orientation.w, goal_msg->pose.orientation.x, goal_msg->pose.orientation.y, goal_msg->pose.orientation.z);
    navigation_goal << goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0,1,2)[2]);
}

pair<double, double> intelligent_plan::pid_twist(pair<double, double> robot_goal_distance_angle,bool is_left){
	if(flag_arriving_direction) return make_pair(0,0); 
	double distance = robot_goal_distance_angle.first;
	double angle = robot_goal_distance_angle.second;
	double cmd_angle = 0,cmd_speed=0;
	if(is_left){
		std::cout << "左边夹角：" << angle << " robot_pose:"<< robot_pose[3] << " navigation_goal:" << navigation_goal << std::endl;
	}else{
		std::cout << "右边夹角：" << angle << " robot_pose:"<< robot_pose[3] << " navigation_goal:" << navigation_goal << std::endl;
	}
	if(flag_arriving_area){
        double delta_angle = robot_pose[3] - navigation_goal[3];
        if(delta_angle>10){
            double cte = 0;
            cte = delta_angle / 180 * M_PI;
			pid_steer.UpdateError(cte);
			cmd_angle = pid_steer.OutputSteerAng();
			cmd_speed = 0;
			return make_pair(cmd_speed,cmd_angle); 
        }else{
            flag_arriving_direction = 1;
			return make_pair(0,0);
        }
		// if(angle>10){
		// 	double cte = 0;
		// 	pid_steer.UpdateError(cte);
		// 	if(is_left){
		// 		cmd_angle = pid_steer.OutputSteerAng();
		// 	}else{
		// 		cmd_angle = -1 * pid_steer.OutputSteerAng();
		// 	}
		// 	cmd_speed = 0;
		// 	return make_pair(cmd_speed,cmd_angle); 
		// }else{
		// 	flag_arriving_direction = 1;
		// 	return make_pair(0,0);
		// }
	}
	if(distance < 0.3)
	{
	    flag_arriving_area = 1;

	}else{
		double cte = 0;
		cte = angle / 180 * M_PI;
		//angle
		pid_steer.UpdateError(cte);
        cmd_angle = pid_steer.OutputSteerAng();
        if(isnan(cmd_angle)) cmd_angle = 0;
		if(!is_left){
			cmd_angle = -1 * pid_steer.OutputSteerAng();
		}
		//speed
		if(angle>30){
			cmd_speed = 0.1;
		}else{
        pid_speed.UpdateError(fabs(cmd_angle));
        cmd_speed = pid_speed.OutputThrottle(_max_throttle);
		if(fabs(cmd_speed)>_max_throttle){
			cmd_speed = _max_throttle;
		}
        if(isnan(cmd_speed)) cmd_speed = 0;
		}
	}
	return make_pair(cmd_speed,cmd_angle); 
}