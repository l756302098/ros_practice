#include "ros/ros.h"
#include "example_service/AddTwoInts.h"
#include <cstdlib>
#include "std_msgs/Int32.h"

ros::ServiceClient client;

void test(const std_msgs::Int32::ConstPtr &msg)
{
    example_service::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 1;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
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
  client = n.serviceClient<example_service::AddTwoInts>("add_two_ints");
  ros::Subscriber sub = n.subscribe("/client/test", 1, &test);

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
  ros::spin();

  return 0;
}