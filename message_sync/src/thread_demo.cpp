#include <ros/ros.h>
#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std;

boost::mutex mutex;
boost::condition_variable_any cond;
std::vector<int> random_numbers;

void make_data()
{
    std::srand(static_cast<unsigned int>(std::time(0)));
    boost::unique_lock<boost::mutex> lock(mutex);
    for (int i = 0; i < 4; ++i)
    {
        cout << "push ..." << endl;
        random_numbers.push_back(std::rand());
        cond.notify_all();
    }
}

void print()
{
    std::size_t next_size = 1;
    while (1)
    {
        boost::unique_lock<boost::mutex> lock(mutex);
        while (random_numbers.size() != 0)
            cond.wait(mutex);
        //读取数据

        //删除数据

        sleep(1);

        cout
            << "print " << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thread_demo");
    ros::NodeHandle nh_;
    ROS_INFO("thread_demo node started...");

    ros::Rate rate(1);
    boost::thread t1(print);
    boost::thread t2(print);
    boost::thread t3(print);
    boost::thread t4(print);
    while (ros::ok())
    {
        ROS_INFO("update...");
        make_data();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
