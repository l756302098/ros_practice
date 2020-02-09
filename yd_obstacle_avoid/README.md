# yd_obstacle_avoid

在 [ros-planning/navigation] (https://github.com/ros-planning/navigation) 的基础上做的智能避障,实现在特定情况下的避障规划.

## Introduction

机器人在任务模式下行走,上一位置到下一位置的间距大于5米并且距离两端点大于1米的情况下,以激光为输入源,结合 navigation 产生的costmap [nav_msgs::OccupancyGrid] 来判断前方是否有障碍物,如果有则进入避障模式开始利用
navigation 的规划来实现行走.

结构图
<img src="https://fbimage.cn/image/other/20204.png" alt="结构图" width="400"/>

## Installation

1.下载对应版本的[navigation] (https://github.com/ros-planning/navigation) 
2.下载本工程到 ros 的工作空间下并解压
3.编译

## Quick start

roslaunch yd_obstacle_avoid intelligent_plan.launch

### Params

以下是参数说明

* **path_save**: system dependent path to folder where all the rosbags will be put: /home/user/rosbag_record 
* **topic**: set of space sperated topics: topic1 topic2 topic3 ...
* **file_name**: the name prefix which will be put before the rosbag time stamp (optional)