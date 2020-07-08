#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "yidamsg/pointcloud_color.h"

#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>
#include "base64.h"

using namespace std;

class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, nav_msgs::Odometry> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *visible_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    std::string odom_topic;
    std::string visible_topic;
    std::string thermal_topic;
    std::string yt_topic;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::Image::ConstPtr &visible_image, const nav_msgs::Odometry::ConstPtr &yt_data);
    void update();
};

message_sync_ros_node::message_sync_ros_node()
{
    ros::param::get("~/odom_topic", odom_topic);
    ros::param::get("~/visible_topic", visible_topic);
    ros::param::get("~/thermal_topic", thermal_topic);
    ros::param::get("~/yt_topic", yt_topic);
    std::cout << "odom_topic:" << odom_topic << std::endl;
    std::cout << "visible_topic:" << visible_topic << std::endl;
    std::cout << "thermal_topic:" << thermal_topic << std::endl;
    std::cout << "yt_topic:" << yt_topic << std::endl;
    rgb_pub = node_.advertise<yidamsg::pointcloud_color>("/yd/pointcloud/vt", 10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic, 1);
    visible_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_, visible_topic, 1);
    yt_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, yt_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *odom_sub_, *visible_sub_, *yt_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2, _3));
}

message_sync_ros_node::~message_sync_ros_node()
{
}

void message_sync_ros_node::update()
{
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::Image::ConstPtr &visible_image, const nav_msgs::Odometry::ConstPtr &yt_data)
{
    // Test start
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(visible_image, sensor_msgs::image_encodings::BGR8);
    // cv::Mat img = cv_ptr->image;

    // vector<uchar> vecImg; //Mat 图片数据转换为vector<uchar>
    // vector<int> vecCompression_params;
    // vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    // vecCompression_params.push_back(90);
    // imencode(".jpg", img, vecImg, vecCompression_params);

    // string imgbase64 = base64_encode(vecImg.data(), vecImg.size()); //实现图片的base64编码
    // cout << imgbase64 << endl;

    // cout << "========================================" << endl;
    // string s_mat = base64_decode(imgbase64.data());
    // std::vector<char> base64_img(s_mat.begin(), s_mat.end());
    // img = cv::imdecode(base64_img, CV_LOAD_IMAGE_COLOR);
    // Test end

    nav_msgs::Odometry current_pose;
    current_pose = *odom_data;

    sensor_msgs::Image v_img;
    v_img = *visible_image;

    nav_msgs::Odometry yt_pose;
    yt_pose = *yt_data;

    yidamsg::pointcloud_color data;
    data.version = 1;
    data.v_format = ".jpg";
    data.v_data = v_img.data;
    //data.pose = pose->pose;

    data.pos_x = current_pose.pose.pose.position.x;
    data.pos_y = current_pose.pose.pose.position.y;
    data.pos_z = current_pose.pose.pose.position.z;
    data.qua_x = current_pose.pose.pose.orientation.x;
    data.qua_y = current_pose.pose.pose.orientation.y;
    data.qua_z = current_pose.pose.pose.orientation.z;
    data.qua_w = current_pose.pose.pose.orientation.w;

    data.horizontal = yt_pose.pose.pose.position.x;
    data.vertical = yt_pose.pose.pose.position.z;

    rgb_pub.publish(data);
    std::cout
        << "callback +1 " << std::endl;
}

void callback1(const sensor_msgs::CompressedImage::ConstPtr &image1, const sensor_msgs::CompressedImage::ConstPtr &image2, const nav_msgs::Odometry::ConstPtr &data)
{
    // Solve all of perception here...
    std::cout << "sync message" << std::endl;
}

void callback2(const sensor_msgs::CompressedImage::ConstPtr &image1, const sensor_msgs::CompressedImage::ConstPtr &image2, const nav_msgs::Odometry::ConstPtr &odom_data, const std_msgs::String::ConstPtr &yuntai_data)
{
    // Solve all of perception here...
    std::cout << "sync message" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    message_sync_ros_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(10);
    while (ros::ok())
    {
        //node.update();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
