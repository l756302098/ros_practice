#include "ros/ros.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <string>

#include <hmi_ui.h>

#include <memory>
#include <iostream>
#include <utility>

using namespace std;
using namespace boost;
using namespace wincomm;

class Foo{

public:
    Foo() = default;
    Foo(int a):_a(a) {}
    ~Foo() {}
    int get_a(){
        return _a;
    }
    void set_a(int a) {
        _a = a;
    }
private:
    int _a;

};

std::unique_ptr<Foo> change_a(std::unique_ptr<Foo> f)
{
    f->set_a(10);
    return f;
}

HmiUI hmi;

void test_node()
{
}

void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    ROS_INFO("callback...");
    diagnostic_msgs::DiagnosticArray array = *msg;
    hmi.updateData(array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi_client");
    ros::NodeHandle nh_;

    ROS_INFO("hmi_client node started...");
    ros::Subscriber  sub = nh_.subscribe<diagnostic_msgs::DiagnosticArray>("/yd/node_status", 1, &callback);
    ros::Rate rate(1);

    while (ros::ok())
    {
        //hmi.freshUI();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}