#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "custom_dynamic_reconfigure/TutorialsConfig.h"

void callback(dynamic_reconfigure::TutorialsConfig &config)
{
  ROS_INFO("Reconfigure Request: %d %f %s %s %d",
           config.int_param,
           config.double_param,
           config.str_param.c_str(),
           config.bool_param?"True":"False",
           config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "custom_dynamic_server");

  dynamic_reconfigure::Server<dynamic_reconfigure::TutorialsConfig> server;
  // boost::recursive_mutex config_mutex;
  // dynamic_reconfigure::Server<dynamic_reconfigure::TutorialsConfig> server(config_mutex);
  dynamic_reconfigure::Server<dynamic_reconfigure::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1); //绑定回调函数
  server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况

  ROS_INFO("Spinning node");
  ros::spin(); //服务器循环监听重配置请求，当服务器收到重配置请求的时候，就会自动调用回调函数

  // while(ros::ok()){
  //   dynamic_reconfigure::TutorialsConfig new_config;
  //   new_config.int_param = 10;
  //   server.updateConfig(new_config);
  //   std::cout << "update config" << std::endl;
  // }
  return 0;
}