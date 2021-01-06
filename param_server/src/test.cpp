#include <ros/ros.h>
#include <thread>
#include "param_server/server.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "TestNode");

  param_server::Server server("param_server","cfg/config.yml");
  // bool value;
  // if(server.exist("boolean-value")){
  //   server.get("boolean-value",value);
  // }
  //std::cout << "readScalar boolean-value:" << value << std::endl;
  // server.set("number-value",50);
  ros::spin();
  // while(ros::ok()){
  //   sleep(1);
  // }

  return 0;
}