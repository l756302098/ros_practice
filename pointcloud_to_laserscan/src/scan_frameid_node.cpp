#include <ros/ros.h>
#include <nodelet/loader.h>
#include <string>

#include <sensor_msgs/LaserScan.h>

std::string raw_topic_, target_topic_, target_frame_;
ros::Subscriber scan_sub;
ros::Publisher scan_pub;

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    //change frame id
    sensor_msgs::LaserScan output;
    output.header = scan_msg->header;
    //output.header.frame_id = target_frame_;
    
    if (!target_frame_.empty())
    {
        output.header.frame_id = target_frame_;
    }
    
    output.angle_min = scan_msg->angle_min;
    output.angle_max = scan_msg->angle_max;
    output.angle_increment = scan_msg->angle_increment;
    output.time_increment = scan_msg->time_increment;
    output.scan_time = scan_msg->scan_time;
    output.range_min = scan_msg->range_min;
    output.range_max = scan_msg->range_max;
    output.ranges = scan_msg->ranges;
    //std::cout << "publish new scan" << std::endl;
    //publish
    scan_pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_frameid_node");

    ros::NodeHandle private_nh;
    private_nh.param<std::string>("/scan_frameid_node/raw_topic", raw_topic_, "/scan");
    private_nh.param<std::string>("/scan_frameid_node/target_topic", target_topic_, "/scan");
    private_nh.param<std::string>("/scan_frameid_node/target_frame", target_frame_, "/scan");
    std::cout << "raw_topic:" << raw_topic_ << std::endl;
    std::cout << "target_topic:" << target_topic_ << std::endl;
    std::cout << "target_frame:" << target_frame_ << std::endl;

    scan_sub = private_nh.subscribe(raw_topic_, 1, scanCallback);
    scan_pub = private_nh.advertise<sensor_msgs::LaserScan>(target_topic_, 1);
    ros::spin();
    return 0;
}
