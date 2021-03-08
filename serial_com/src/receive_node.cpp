/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-01-08 13:56:17
 * @LastEditors: li
 * @LastEditTime: 2021-03-05 15:40:09
 */
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <string>
#include <iostream>
#include <thread>
#include "std_msgs/String.h"
#include "serial_com/async_serial.h"
#include "urgent.pb.h"
#include <vector>
#include <google/protobuf/util/time_util.h>

using namespace std;
using namespace boost::asio;
using google::protobuf::util::TimeUtil;

class Foo
{
public:
  vector<char> buff;
  
	void received(const char *data, unsigned int len)
	{
		string s(data,len);
    //std::cout << "receive:" << s << std::endl;
    for(int i=0;i<len;i++){
      buff.push_back(*data);
      data++;
    }
    if(strcmp(s.c_str(),"\n")==0){
      string result;
      result.assign(buff.begin(), buff.end());
      ydpb::Urgent msg;
      msg.ParseFromString(result);
      std::cout << "receive one message! result:" << msg.tid() << std::endl;
      if (msg.has_last_updated()) {
        std::cout << "  Updated: " << TimeUtil::ToString(msg.last_updated()) << endl;
      }
      buff.clear();
      //result.clear();
    }
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle n;
  std::string ttyName = "";
  ros::param::get("/receive_node/tty_name",ttyName);
  std::cout << "ttyName:" << ttyName << std::endl;

  Foo foo;
  CallbackAsyncSerial serial(ttyName,115200);
  serial.setCallback(bind(&Foo::received,foo,std::placeholders::_1,std::placeholders::_2));
  ros::Rate rate(10);

  while (ros::ok())
  {
    //serial.writeString("Hello world\n");
	  ros::spinOnce();
    rate.sleep();
  }
  serial.clearCallback();

  return 0;
}
