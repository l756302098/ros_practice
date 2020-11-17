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
	nh.param("isvalid_path", _valid_path, true);
	nh.param("pose_topic", _pose_topic,	std::string("/odom_localization"));
	nh.param("cmd_topic", _cmd_topic,	std::string("/mobile_base/commands/velocity"));
	nh.param("angle_kp", _angule_kp, 0.1);
	nh.param("angle_ki", _angule_ki, 0.001);
	nh.param("angle_kd", _angule_kd, 2.8);
	nh.param("speed_kp", _speed_kp, 0.0);
	nh.param("speed_ki", _speed_ki, 0.0);
	nh.param("speed_kd", _speed_kd, 0.0);
	nh.param("max_throttle", _max_throttle, 0.5);
	nh.param("goal_throttle", _goal_throttle, 0.3);
	std::cout << "path_topic:" << _path_topic << " pose_topic:"<<_pose_topic  << std::endl;
	std::cout << "angle_kp:" << _angule_kp << " angle_ki:" << _angule_ki << " angle_kd:"<< _angule_kd << std::endl;
	std::cout << "speed_kp:" << _speed_kp << " speed_ki:" << _speed_ki << " speed_kd:"<< _speed_kd << " max_throttle："<<_max_throttle << std::endl;
 	ROS_INFO("intelligent_plan init");
    connect_server();
    flag_goal = 0;
	flag_path = 0;
	flag_arriving_area = 0;
	flag_arriving_direction = 0;
	goal_seq = 0;
	pid_steer.Init(_angule_kp, _angule_ki, _angule_kd);
	pid_speed.Init(_speed_kp, _speed_ki, _speed_kd);
    //sub
    test_sub = nh.subscribe("/walle/walk/cancel", 1, &intelligent_plan::test, this);
	planing_result_pub = nh.advertise<std_msgs::Bool>("/walle/walk/done", 5, true);
    //server
    task_service = nh.advertiseService("/walle/walk/goal", &intelligent_plan::task_service_cb,this);
    //move_base
	path_sub = nh.subscribe(_path_topic, 1, &intelligent_plan::path_callback, this);
	pose_sub = nh.subscribe(_pose_topic, 1, &intelligent_plan::pose_callback, this);
	cart_pose_sub = nh.subscribe("/cart/pose", 1, &intelligent_plan::cart_pose_callback, this);
	click_sub = nh.subscribe("/clicked_point", 1, &intelligent_plan::click_callback, this);
	cmd_pub = nh.advertise<geometry_msgs::Twist>(_cmd_topic, 5, true);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &intelligent_plan::goal_callback, this);
	simple_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
	cancle_pub =  nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
}

intelligent_plan::~intelligent_plan()
{
    
}

void intelligent_plan::connect_server()
{
    client = new Client("move_base", true);
    //等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

void intelligent_plan::update()
{
    if(intelligent_plan::plan_stage == PlanStage::NONE) return;
	if(intelligent_plan::plan_stage == PlanStage::SUCCEEDED){
		std_msgs::Bool result;
		result.data = true;
		planing_result_pub.publish(result);
		intelligent_plan::plan_stage = PlanStage::NONE;
	}else if(intelligent_plan::plan_stage == PlanStage::FAILED){
		std_msgs::Bool result;
		result.data = false;
		planing_result_pub.publish(result);
	}else if(intelligent_plan::plan_stage == PlanStage::ACTIVE){
		
	}else{
		std::cout << "navigation state:" << intelligent_plan::plan_stage << std::endl;
	}
}

void intelligent_plan::test(const std_msgs::Bool::ConstPtr &msg)
{
	ROS_INFO("cancel goal");
	is_cancel = true;
	flag_goal = 0;
    flag_arriving_area = 0;
    flag_arriving_direction = 0;
	//TODD:publish cancel
	if(client!=nullptr){
		client->cancelAllGoals();
	}
	// actionlib_msgs::GoalID empty_goal;
	// cancle_pub.publish(empty_goal);
}

bool intelligent_plan::task_service_cb(yidamsg::wali_go_to_position::Request &req,yidamsg::wali_go_to_position::Response &res){
    geometry_msgs::Pose new_goal;
    new_goal.position.x = req.pose.position.x;
    new_goal.position.y = req.pose.position.y;
    new_goal.position.z = req.pose.position.z;
    new_goal.orientation.x = req.pose.orientation.x;
    new_goal.orientation.y = req.pose.orientation.y;
    new_goal.orientation.z = req.pose.orientation.z;
    new_goal.orientation.w = req.pose.orientation.w;
	bool isvalid = isvalid_goal(new_goal);
	if(!isvalid){
		ROS_ERROR("goal orientation is not valid");
		res.success = false;
		return true;
	}
	//开始task
	is_cancel = false;
    flag_goal = 1;
    flag_arriving_area = 0;
    flag_arriving_direction = 0;
	//save goal
	Quaternionf quanternion = Quaternionf(new_goal.orientation.w, new_goal.orientation.x, new_goal.orientation.y, new_goal.orientation.z);
    navigation_goal << new_goal.position.x, new_goal.position.y, new_goal.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0,1,2)[2]);

	// geometry_msgs::PoseStamped simple_goal;
	// simple_goal.header.seq = goal_seq + 1;
	// simple_goal.header.frame_id = "map";
	// simple_goal.header.stamp = ros::Time::now();
	// simple_goal.pose = new_goal; 
	// simple_goal_pub.publish(simple_goal);
    move_base_msgs::MoveBaseGoal base_goal;
    base_goal.target_pose.header.stamp = ros::Time::now();
    base_goal.target_pose.header.frame_id = "map";
    base_goal.target_pose.pose = new_goal;
    client->sendGoal(base_goal, &intelligent_plan::doneCb, &intelligent_plan::activeCb, &intelligent_plan::feedbackCb);
	res.success = true;
    return true;
}

void intelligent_plan::path_callback(const nav_msgs::PathConstPtr& path_msg){
	if(flag_goal){
		if(_valid_path){
			bool isvalid = isvalid_path(path_msg);
			if(!isvalid) return;
		}
		int index = (20<path_msg->poses.size())?20:path_msg->poses.size();
		//geometry_msgs::PoseStamped[] opath = path_msg->pose[];
		if(index>1){
			geometry_msgs::PoseStamped goal = path_msg->poses[index-1];
			next_goal[0] = goal.pose.position.x;
			next_goal[1] = goal.pose.position.y;
			next_goal[2] = goal.pose.position.z;
		}
		flag_path = 10;
		// path = *path_msg;
	    // int path_size = (100<path_msg->poses.size())?100:path_msg->poses.size();
	    // next_goal = Vector3f(0, 0, 0);
	    // for(int i=0; i<path_size; i++)
	    // {
	    //     next_goal[0] += path.poses[i].pose.position.x;
	    //     next_goal[1] += path.poses[i].pose.position.y;
	    //     next_goal[2] += path.poses[i].pose.position.z;
	    // }
	    // next_goal = next_goal / path_size;
	    // flag_path = 20;
	}    
}

void intelligent_plan::cart_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg){
	if(is_cancel){
		publishZero();
	}
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
		ROS_INFO("published v:%f and w:%f",twist.first,twist.second);
	    geometry_msgs::Twist motor_control;
	    motor_control.linear.x = twist.first;
	    motor_control.angular.z = twist.second;
	    cmd_pub.publish(motor_control);
	    //cout << "published v and w is: " << twist.first << "  " << twist.second << endl;
    }else{
		publishZero();
	}
}

void intelligent_plan::publishZero(){
	geometry_msgs::Twist motor_control;
	motor_control.linear.x = 0;
	motor_control.angular.z = 0;
	cmd_pub.publish(motor_control);
}

void intelligent_plan::pose_callback(const nav_msgs::OdometryConstPtr& pose_msg){
	if(is_cancel){
		publishZero();
	}
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
		//std::cout << "published v and w is: " << twist.first << "  " << twist.second << std::endl;
		ROS_INFO("published v:%f and w:%f",twist.first,twist.second);
	    geometry_msgs::Twist motor_control;
	    motor_control.linear.x = twist.first;
	    motor_control.angular.z = twist.second;
	    cmd_pub.publish(motor_control);
    }else{
		publishZero();
	}
}
void intelligent_plan::click_callback(const geometry_msgs::PointStampedConstPtr& goal_msg){
	//new goal

}

void intelligent_plan::goal_callback(const geometry_msgs::PoseStampedConstPtr& goal_msg){
	is_cancel = false;
    flag_goal = 1;
    flag_arriving_area = 0;
    flag_arriving_direction = 0;
	goal_seq = goal_msg->header.seq;
    Quaternionf quanternion = Quaternionf(goal_msg->pose.orientation.w, goal_msg->pose.orientation.x, goal_msg->pose.orientation.y, goal_msg->pose.orientation.z);
    navigation_goal << goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0,1,2)[2]);
}

pair<double, double> intelligent_plan::pid_twist(pair<double, double> robot_goal_distance_angle,bool is_left){
	if(flag_arriving_direction) return make_pair(0,0); 
	double distance = robot_goal_distance_angle.first;
	double angle = robot_goal_distance_angle.second;
	double cmd_angle = 0,cmd_speed=0;
	if(is_left){
		std::cout << "左边夹角：" << angle << " distance:" <<distance << " robot_pose:"<< robot_pose[3] << " navigation_goal:" << navigation_goal[3] << std::endl;
	}else{
		std::cout << "右边夹角：" << angle << " distance:" <<distance <<" robot_pose:"<< robot_pose[3] << " navigation_goal:" << navigation_goal[3] << std::endl;
	}
	if(flag_arriving_area){
        double delta_angle = robot_pose[3] - navigation_goal[3];
		std::cout << "delta goal angle:" << delta_angle << std::endl;
        if(fabs(delta_angle)>10){
			//turn left or right
			cmd_angle = 0.2;
			return make_pair(cmd_speed,cmd_angle); 
        }else{
			ROS_INFO("arriving angle");
			//publish
			std_msgs::Bool result;
			result.data = true;
			planing_result_pub.publish(result);
			//cancel goal
			actionlib_msgs::GoalID empty_goal;
			cancle_pub.publish(empty_goal);
            flag_arriving_direction = 1;
			return make_pair(0,0);
        }
	}
	if(distance < _goal_throttle)
	{
		ROS_INFO("distance goal < 0.3");
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

bool intelligent_plan::isvalid_path(const nav_msgs::PathConstPtr& path_msg){
	if(path_msg->poses.size()>1){
		int index = path_msg->poses.size() - 1;
		//geometry_msgs::PoseStamped[] opath = path_msg->pose[];
		geometry_msgs::PoseStamped goal = path_msg->poses[index];
		float goal_x = goal.pose.position.x;
		float goal_y = goal.pose.position.y;
		float distance = sqrt(pow(navigation_goal[0]-goal_x, 2) + pow(navigation_goal[1]-goal_y, 2));
		std::cout << "path_x:" << goal_x << " path_y:" <<  goal_y<< " goal_x:" << navigation_goal[0] << " goal_y:"<< navigation_goal[1] << " distance:"<< distance<<std::endl;
		if(distance < 0.1) return true;
	}
	return false;
}

bool intelligent_plan::isvalid_goal(const geometry_msgs::Pose& pose){
	//first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
}