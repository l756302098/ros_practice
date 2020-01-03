#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include "std_srvs/SetBool.h"
#include <stdio.h>
#include <string.h>
#include <syslog.h>

using namespace std;

std::string topic, file_name;
int timeout_num;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hdl_daemon");
    ros::NodeHandle n;
    n.param<std::string>("/hdl_daemon/topic", topic, "/hdl/heartbeat");
    n.param<int>("/hdl_daemon/timeout_num", timeout_num, 5);
    n.param<std::string>("/hdl_daemon/file_name", file_name, "");

    cout << "topic" << topic << endl;
    cout << "timeout_num" << timeout_num << endl;
    cout << "file_name" << file_name << endl;

    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>(topic);
    ros::Rate loop_rate(1);

    int count = -120;
    while (ros::ok())
    {
        std_srvs::SetBool srv;
        if (client.call(srv))
        {
            cout << "hdl respose : true" << endl;
            count = 0;
        }
        else
        {
            syslog(LOG_INFO, "连续%d次无响应", count);
            cout << "Failed to call service add_two_ints" << count << endl;
            count++;
            if (count >= timeout_num)
            {
                cout << "-----------restart node-----------" << count << endl;
                syslog(LOG_INFO, "重启电脑！！！！！！！！！！！");

                FILE *fp;
                char buffer[80];
                memset(buffer, 0x00, sizeof(buffer));

                fp = popen(file_name.c_str(), "r");
                fgets(buffer, sizeof(buffer), fp);

                printf("[%s]\n", buffer);

                pclose(fp);

                count = -300;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}