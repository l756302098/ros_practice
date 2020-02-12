#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <queue>

#include <boost/thread.hpp>

#ifndef PC2LS_VALUES_H_
#define PC2LS_VALUES_H_

class pc2ls
{
private:
    /* data */
    boost::mutex mutex;
    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Subscriber sub;

    std::string target_frame_, raw_frame_,
        cloud_topic_, scan_topic_, transfrom_frame_;
    double tolerance_, min_height_, max_height_, angle_min_, angle_max_, angle_increment_;
    double scan_time_, range_min_, range_max_, inf_epsilon_;
    bool use_inf_, is_get_transfrom;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::queue<sensor_msgs::PointCloud2ConstPtr> pq;

public:
    pc2ls(/* args */);
    ~pc2ls();
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void deal_queue();
};

#endif
