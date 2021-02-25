#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <string>
#include <iostream>
#include <thread>
#include "std_msgs/String.h"
#include "serial_com/async_serial.h"
#include "urgent.pb.h"

using namespace std;
using namespace boost::asio;

class Foo
{
public:
	void received(const char *data, unsigned int len)
	{
		string s(data,len);
		cout<<"Callback called! \""<<s<<"\""<<endl;
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle n;
  std::string ttyName;
  ros::param::get("/send_node/tty_name",ttyName);
  std::cout << "ttyName:" << ttyName << std::endl;

  Foo foo;
  CallbackAsyncSerial serial(ttyName,115200);
  serial.setCallback(bind(&Foo::received,foo,std::placeholders::_1,std::placeholders::_2));
  ros::Rate rate(1);
  ydpb::Urgent msg;
  msg.add_temp(70.0);
  msg.add_temp(60.0);
  msg.add_temp(50.0);
  msg.add_temp(40.0);
  ydpb::Pos* position = msg.add_cpos();
  position->set_pos_x(0);
  position->set_pos_y(0);
  position->set_pos_z(0);
  position->set_qua_x(0);
  position->set_qua_y(0);
  position->set_qua_z(0);
  position->set_qua_w(1);
  for(int i=0;i<30;i++){
      ydpb::Pos* temp = msg.add_hpos();
      temp->set_pos_x(0);
      temp->set_pos_y(0);
      temp->set_pos_z(0);
      temp->set_qua_x(0);
      temp->set_qua_y(0);
      temp->set_qua_z(0);
      temp->set_qua_w(1);
  }
  msg.set_bat(1.0);
  msg.set_tid(152);
  msg.set_err(10011);
  std::string output = "";
  if(msg.SerializeToString(&output)){
      std::cout << "SerializeToString success! output size:" << sizeof(output) << std::endl;
  }
  
  //fstream output("./log", ios::out | ios::trunc | ios::binary);
  //if (!msg.SerializeToOstream(&output)) { cerr << "Failed to write msg." << endl; return -1; }
  //std::cout << "write success!" << std::endl;
  while (ros::ok())
  {
    //serial.writeString("Hello world\n");
    serial.writeString(output+"\n");
	  ros::spinOnce();
    rate.sleep();
  }
  serial.clearCallback();

  return 0;
}
