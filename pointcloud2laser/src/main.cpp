#include "ros/ros.h"
#include "std_msgs/String.h"
#include <limits>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub_;
ros::Subscriber sub;

std::string target_frame_, raw_frame_,
    cloud_topic, scan_topic;
double tolerance_, min_height_, max_height_, angle_min_, angle_max_, angle_increment_;
double scan_time_, range_min_, range_max_, inf_epsilon_;
bool use_inf_, is_get_transfrom;

boost::shared_ptr<tf2_ros::Buffer> tf2_;
boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void pointCB(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    ROS_INFO("I get message");

    sensor_msgs::LaserScan output;
    output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
        output.header.frame_id = target_frame_;
    }
    //output.header.stamp = ros::Time::now();

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
        output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
        output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
    }

    sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;
    ros::Time begin = ros::Time::now();
    // Transform cloud if necessary
    if (!(output.header.frame_id == cloud_msg->header.frame_id))
    {
        try
        {
            cloud.reset(new sensor_msgs::PointCloud2);
            tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(2));
            cloud_out = cloud;
            // tf2::doTransform(*cloud_msg, *cloud, transformStamped);
            // cloud_out = cloud;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_INFO("Transform failure:%s ", ex.what());
            return;
        }
    }
    else
    {
        cloud_out = cloud_msg;
    }
    ros::Time level1 = ros::Time::now();
    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
         iter_z(*cloud_out, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
            continue;
        }

        if (*iter_z > max_height_ || *iter_z < min_height_)
        {
            ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
            continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min_)
        {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                      *iter_y, *iter_z);
            continue;
        }
        if (range > range_max_)
        {
            ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                      *iter_y, *iter_z);
            continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            continue;
        }

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - output.angle_min) / output.angle_increment;
        if (range < output.ranges[index])
        {
            output.ranges[index] = range;
        }
    }
    ros::Time level2 = ros::Time::now();
    ros::Duration time1 = level1 - begin;
    ros::Duration time2 = level2 - level1;
    std::cout << "time1:" << time1 << " time2:" << time2 << std::endl;
    //std::cout << "pub laser" << std::endl;
    pub_.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point2laser");
    ros::NodeHandle private_nh_;

    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    private_nh_.param<std::string>("/pointcloud2laser/target_frame", target_frame_, "");
    private_nh_.param<std::string>("/pointcloud2laser/raw_frame", raw_frame_, "");
    private_nh_.param<double>("/pointcloud2laser/transform_tolerance", tolerance_, 0.0);
    private_nh_.param<double>("/pointcloud2laser/min_height", min_height_, std::numeric_limits<double>::min());
    private_nh_.param<double>("/pointcloud2laser/max_height", max_height_, std::numeric_limits<double>::max());

    private_nh_.param<double>("/pointcloud2laser/angle_min", angle_min_, -M_PI);
    private_nh_.param<double>("/pointcloud2laser/angle_max", angle_max_, M_PI);
    private_nh_.param<double>("/pointcloud2laser/angle_increment", angle_increment_, M_PI / 180.0);
    private_nh_.param<double>("/pointcloud2laser/scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("/pointcloud2laser/range_min", range_min_, 0.0);
    private_nh_.param<double>("/pointcloud2laser/range_max", range_max_, std::numeric_limits<double>::max());
    private_nh_.param<double>("/pointcloud2laser/inf_epsilon", inf_epsilon_, 1.0);

    private_nh_.param<std::string>("/pointcloud2laser/cloud_topic", cloud_topic, "cloud");
    private_nh_.param<std::string>("/pointcloud2laser/scan_topic", scan_topic, "scan");

    private_nh_.param<bool>("/pointcloud2laser/use_inf", use_inf_, true);

    std::cout << "cloud_topic" << cloud_topic << std::endl;
    std::cout << "scan_topic" << scan_topic << std::endl;

    sub = private_nh_.subscribe(cloud_topic, 1, pointCB);
    pub_ = private_nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 10);

    // ros::Rate rate(5);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ros::spin();
    return 0;
}
