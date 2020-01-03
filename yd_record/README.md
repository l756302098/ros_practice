# 说明 
在 yd_record/launch/yd_record.launch,需要指定生成目录
开启后记录指定topic,由于生成的bag比较大，时间周期不宜太大,程序只会保留最近两次的bag,当发现程序异常时，发送停止命令终止程序。

## start
roslaunch yd_record yd_record.launch

## stop
rostopic pub -1 /yd_record/cmd std_msgs/Int16 "data: 0"

## launch
launch 中的参数
path_save 存放路径
topic 需要监听的topic
seconds 记录周期（秒）













