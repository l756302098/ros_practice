# robot_monitor
监控机器人定位的性能  
## 1.监听定位发送频率  
## 2.监听定位 buffer size  
当size 大于阈值发送停止命令  
当size 小于阈值发送开始命令  
当size 大于处理极限（clear size）时清理buffer size  
## 3.使用 syslog 输出日志到 /var/log/syslog 方便统一查看
## 启动
roslaunch robot_monitor robot_monitor.launch

# robot_daemon
进程守护
