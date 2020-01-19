# ros_practice

Here are some of the functions I use in my work.

# Table of Contents

* [PID]
    * [PControl.py]   P 控制器
    * [PDControl.py]   PD 控制器
    * [PIDControl.py]   PID 控制器
    * [Param.py]   局部爬山算法调参
    

* [robot_monitor]
  用于监控机器人的各项性能
  
* [cloud_platform]
  用于操控云台
  
* [yd_record]
  用于记录topic的数据
  
* [message_sync]
  ros系统下同步多个topic
  
* [pointcloud_to_laser]
    * [pointcloud_to_laserscan/pointcloud_to_laserscan]   点云转激光
    * [pointcloud_to_laserscan/laserscan_to_pointcloud]   激光转点云
    * [pointcloud_to_laserscan/scan_frame]    修改scan 的frame_id
    
* [ntp]
  在Ubuntu下利用ntp 实现局域网内时间同步
  
* [irqbalance]
  irqbalance 利用irqbalance 优化系统的软终断
  
* [robot_odometer]
  在ros 下读取串口数据，并发送到 ros master，未完...
  
* [path_sim]
  结合 navigation stack 实现检测障碍物
  
# PID
## PControl
<img src="https://fbimage.cn/image/other/Figure_1.png" alt="PControl" width="400"/>

## PDControl
<img src="https://fbimage.cn/image/other/Figure_2.png" alt="PDControl" width="400"/>

## PIDControl
<img src="https://fbimage.cn/image/other/Figure_3.png" alt="PIDControl" width="400"/>

## 调整参数后
<img src="https://fbimage.cn/image/other/Figure_4.png" alt="PControl" width="400"/>
