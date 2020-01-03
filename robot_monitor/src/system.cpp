// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h> // sleep

using namespace std;

// in seconds
const static unsigned int INTERVAL = 1;

class CPUPercentage
{
public:
    CPUPercentage();
    ~CPUPercentage();

    const float &get_percentage();

private:
    ifstream m_stat_file;
    unsigned long long m_current_user;
    unsigned long long m_current_system;
    unsigned long long m_current_nice;
    unsigned long long m_current_idle;
    unsigned long long m_next_user;
    unsigned long long m_next_system;
    unsigned long long m_next_nice;
    unsigned long long m_next_idle;
    unsigned long long m_diff_user;
    unsigned long long m_diff_system;
    unsigned long long m_diff_nice;
    unsigned long long m_diff_idle;

    string m_stat_line;
    size_t m_line_start_pos;
    size_t m_line_end_pos;
    istringstream m_iss;

    float m_percentage;
};

CPUPercentage::CPUPercentage() : m_current_user(0),
                                 m_current_system(0),
                                 m_current_nice(0),
                                 m_current_idle(0),
                                 m_next_user(0),
                                 m_next_system(0),
                                 m_next_nice(0),
                                 m_next_idle(0),
                                 m_diff_user(0),
                                 m_diff_system(0),
                                 m_diff_nice(0),
                                 m_diff_idle(0)
{
    m_stat_file.exceptions(ifstream::eofbit | ifstream::failbit | ifstream::badbit);
}

CPUPercentage::~CPUPercentage()
{
    if (m_stat_file.is_open())
        m_stat_file.close();
}

const float &CPUPercentage::get_percentage()
{
    m_stat_file.open("/proc/stat");
    getline(m_stat_file, m_stat_line);
    m_stat_file.close();

    // skip "cpu"
    m_line_start_pos = m_stat_line.find_first_not_of(" ", 3);
    m_line_end_pos = m_stat_line.find_first_of(' ', m_line_start_pos);
    m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
    m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
    m_line_end_pos = m_stat_line.find_first_of(' ', m_line_end_pos + 1);
    m_iss.str(m_stat_line.substr(m_line_start_pos, m_line_end_pos - m_line_start_pos));
    m_iss >> m_next_user >> m_next_nice >> m_next_system >> m_next_idle;
    m_iss.clear();

    m_diff_user = m_next_user - m_current_user;
    m_diff_system = m_next_system - m_current_system;
    m_diff_nice = m_next_nice - m_current_nice;
    m_diff_idle = m_next_idle - m_current_idle;
    m_percentage = static_cast<float>(m_diff_user + m_diff_system + m_diff_nice) / static_cast<float>(m_diff_user + m_diff_system + m_diff_nice + m_diff_idle) * 100.0;

    m_current_user = m_next_user;
    m_current_system = m_next_system;
    m_current_nice = m_next_nice;
    m_current_idle = m_next_idle;

    return m_percentage;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system");
    ROS_INFO_NAMED("system", "Hello %s", "World");
    float percentage = 0.0;
    CPUPercentage cpu_per;

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("system/info", 10);

    ros::Rate loop_rate(2);

    while (ros::ok())
    {
        percentage = cpu_per.get_percentage();
        std_msgs::String msg;

        std::stringstream ss;

        ss << percentage;

        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
