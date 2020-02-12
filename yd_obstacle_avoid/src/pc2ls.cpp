#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <yd_obstacle_avoid/pc2ls.h>

pc2ls::pc2ls(/* args */)
{
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    nh.param<std::string>("/intelligent_plan_node/target_frame", target_frame_, "");
    nh.param<std::string>("/intelligent_plan_node/transfrom_frame", transfrom_frame_, "");
    nh.param<std::string>("/intelligent_plan_node/cloud_topic", cloud_topic_, "");
    nh.param<std::string>("/intelligent_plan_node/scan_topic", scan_topic_, "");

    nh.param<double>("/intelligent_plan_node/transform_tolerance", tolerance_, 0.01);
    nh.param<double>("/intelligent_plan_node/min_height", min_height_, 0);
    nh.param<double>("/intelligent_plan_node/max_height", max_height_, 10);

    nh.param<double>("/intelligent_plan_node/angle_min", angle_min_, -M_PI);
    nh.param<double>("/intelligent_plan_node/angle_max", angle_max_, M_PI);
    nh.param<double>("/intelligent_plan_node/angle_increment", angle_increment_, M_PI / 180.0);
    nh.param<double>("/intelligent_plan_node/scan_time", scan_time_, 1.0 / 30.0);
    nh.param<double>("/intelligent_plan_node/range_min", range_min_, 0.0);
    nh.param<double>("/intelligent_plan_node/range_max", range_max_, std::numeric_limits<double>::max());
    nh.param<double>("/intelligent_plan_node/inf_epsilon", inf_epsilon_, 1.0);

    std::cout << "cloud_topic:" << cloud_topic_ << std::endl;
    std::cout << "scan_topic:" << scan_topic_ << std::endl;

    sub = nh.subscribe(cloud_topic_, 1, &pc2ls::cloudCb,this);
    pub_ = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);
}

pc2ls::~pc2ls()
{
}

void pc2ls::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    //add list
    mutex.lock();
    pq.push(cloud_msg);
    mutex.unlock();
}

void pc2ls::deal_queue(){
    if(pq.empty())
        return;
    //get from queue
    mutex.lock();
    sensor_msgs::PointCloud2ConstPtr cloud_msg = pq.back();
    for (int i = 0; i < pq.size(); i++)
    {
        pq.pop();
    }

    mutex.unlock();
    //deal data
    sensor_msgs::LaserScan output;
    output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
        output.header.frame_id = target_frame_;
    }

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

    // Transform cloud if necessary
    if (!(transfrom_frame_ == cloud_msg->header.frame_id))
    {
        try
        {
            cloud.reset(new sensor_msgs::PointCloud2);
            tf2_->transform(*cloud_msg, *cloud, transfrom_frame_, ros::Duration(1));
            cloud_out = cloud;
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

    pub_.publish(output);
}