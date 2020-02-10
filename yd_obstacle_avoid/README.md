# yd_obstacle_avoid

在 [ros-planning/navigation](https://github.com/ros-planning/navigation) 的基础上做的智能避障,实现在特定情况下的避障规划.

## Introduction

机器人在任务模式下行走,上一位置到下一位置的间距大于5米并且距离两端点大于1米的情况下,以激光为输入源,结合 navigation 产生的 costmap 来判断前方是否有障碍物,如果有则进入避障模式开始利用
navigation 的规划来实现行走.

<img src="https://fbimage.cn/image/other/20204.png" alt="结构图" width="600"/>

## Installation

1.下载对应版本的 [navigation](https://github.com/ros-planning/navigation)   
2.下载本工程到 ros 的工作空间下并解压  
3.编译  

## Prepare  

1.准备地图文件,放置在 ./yd_obstacle_avoid/maps/ 下,并修改 navigation.launch 中的map 参数
2.测试地图文件是否可用  

* **启动**: roslaunch yd_obstacle_avoid navigation.launch  
* **查看**: rviz 中查看地图是否正确

## Quick start

roslaunch yd_obstacle_avoid intelligent_plan.launch

### Subscribed Topics

1./move_base/global_costmap/costmap (geometry_msgs/PoseStamped)  
2./odom_localization (nav_msgs::Odometry)  
3./task_status (yidamsg/task_status)  

### Published Topics

1./yida/robot/control_model (std_msgs::Int32)  
2./yida/obstacle_avoid/result (std_msgs::Int32)  
3./yida/hearbeat (diagnostic_msgs::DiagnosticArray)  

### Params

参数说明  

* **localmap_topic**: 订阅的 navigation costmap 的 topic  
* **odom_topic**: 订阅的 navigation 中 costmap 的 topic  
* **task_topic**: 订阅的 task manager 中任务状态的 topic  
* **robot_width**: 机器人的宽度  
* **detection_length**: 检测的道路长度  
* **road_min**: 检测的两点之间的最短距离  
