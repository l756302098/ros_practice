#include "record_ros/record.h"
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

Record::Record(ros::NodeHandle &nh, rosbag::RecorderOptions const &options) : rosbag::Recorder(options)
{
    service_srv = nh.advertiseService("cmd", &Record::string_command, this);
    b_record = true;
}

void Record::wait_for_callback()
{
    ros::Rate r(1); // 60 hz
    while (!b_record && ros::ok())
    {
        ROS_INFO_STREAM("wait_for_callback");
        ros::spinOnce();
        r.sleep();
    }
}

bool Record::string_command(record_ros::String_cmd::Request &req, record_ros::String_cmd::Response &res)
{
    std::string cmd = req.cmd;
    ROS_INFO("Record callback");
    if (cmd == "start")
    {
        if (b_record)
        {
            ros::shutdown();
            res.res = "stopping recorder";
        }
        else
        {
            b_record = true;
            res.res = "starting recorder";
        }
        cmd_stop = false;
        return true;
    }
    else if (cmd == "stop")
    {
        ros::shutdown();
        res.res = "stopping recorder";
        //delete
        cmd_stop = true;
        return true;
    }
    else
    {
        res.res = "No such command[" + cmd + "] in [Record::string_command]";
        ROS_WARN_STREAM(res.res);
        return false;
    }
}
