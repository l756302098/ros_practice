#include "ros/ros.h"
#include "example_service/AddTwoInts.h"
#include <cstdlib>
#include "std_msgs/Int32.h"
#include <boost/thread.hpp>

ros::ServiceClient client;

void test(const std_msgs::Int32::ConstPtr &msg)
{
  example_service::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 1;
  bool isValid = client.isValid();
  bool isPersistent = 	client.isPersistent();
  ROS_INFO("isValid:%i isPersistent:%i",isValid,isPersistent);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

void* print1(void* data){
    ros::NodeHandle *node = (ros::NodeHandle*)data;
    ros::Subscriber sub = node->subscribe<std_msgs::Int32>("/client/test", 2, boost::bind(&test,_1));
    ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  client = n.serviceClient<example_service::AddTwoInts>("add_two_ints",1);
  
  pthread_t ntid;
  int err = pthread_create(&ntid, NULL, print1, (void*)&n);

  example_service::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}