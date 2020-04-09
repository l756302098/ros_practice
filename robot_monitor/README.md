# robot_monitor


## Introduction

监控机器人定位的性能  

1.监听定位 buffer size  
当size 大于阈值(threshold_buffer)发送停止(stop_hdl_topic)命令  
当size 小于阈值(threshold_buffer)发送开始(stop_hdl_topic)命令  
当size 大于处理极限(clear_buff_size)时清理buffer size (clear_buff_topic)  
当size 连续多次(threshold_clear_count)清理定位休眠1分钟(laster_sleep_time)  
2.通过service 获取hdl 心跳，如果连续多次没有相应重启Xavier  
修改restart.sh中 '123' 中123为本机密码  
3.使用 syslog 输出日志到 /var/log/syslog 方便统一查看  

## Quick start
roslaunch robot_monitor robot_monitor.launch

### Params

参数说明  

* **robotpose_topic**: 订阅的 定位 的 topic  
* **threshold_buffer**: 停止定位的 laser buffer 的阈值   
* **clear_buff_size**: 超过size清除laser buffer    
* **control_topic**: 停止机器人运动的topic  
* **buffer_topic**: laser buffer 的topic  
* **clear_buff_topic**: 连续清除buffer 的阈值  
* **stop_hdl_topic**: 停止定位的topic  
* **threshold_clear_count**: 连续 clear buffer 的最大值,超过休眠  
* **laster_sleep_time**: 休眠时间（秒）  
* **file_name**: 重启电脑的脚本名字（～/restart.sh）  
