#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "yidamsg/pointcloud_color.h"
#include "std_msgs/UInt8.h"

#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>
#include "base64.h"

//#include <unistd.h>
#include <stdio.h>
#include <termios.h>

//#include <keyboard/Key.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

using namespace std;

class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::CompressedImage> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *thermal_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    std::string odom_topic;
    std::string visible_topic;
    std::string thermal_topic;
    std::string yt_topic;
    int count, scount;
    bool is_record;
    ofstream map_points;
    std::string root_path;
    std::string visible_path;
    std::string thermal_path;
    string visible_image_name;
    string thermal_image_name;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &thermal_image);
    void update();
    void isDirectory(string path);
};

message_sync_ros_node::message_sync_ros_node()
{
    node_.param<std::string>("/sync_thermal/odom_topic", odom_topic, "/odom_localization");
    node_.param<std::string>("/sync_thermal/thermal_topic", thermal_topic, "/thermal/image_proc/compressed");
    std::cout << "odom_topic:" << odom_topic << std::endl;
    std::cout << "thermal_topic:" << thermal_topic << std::endl;

    isDirectory(root_path);
    visible_path = root_path + "/gray";
    isDirectory(visible_path);
    thermal_path = root_path + "/thermal";
    isDirectory(thermal_path);
    string record_path = root_path + "/record.txt";
    map_points.open(record_path, ios_base::app);

    rgb_pub = node_.advertise<yidamsg::pointcloud_color>("/yd/pointcloud/vt", 10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic, 1);
    thermal_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, thermal_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *odom_sub_, *thermal_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2));
}

message_sync_ros_node::~message_sync_ros_node()
{
    map_points.close();
}

void message_sync_ros_node::update()
{
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &thermal_image)
{
    nav_msgs::Odometry current_pose;
    current_pose = *odom_data;

    sensor_msgs::CompressedImage t_img;
    t_img = *thermal_image;

    yidamsg::pointcloud_color data;

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);

    data.t_format = t_img.format;
    cv_bridge::CvImagePtr t_cv_ptr = cv_bridge::toCvCopy(t_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat t_m_img = t_cv_ptr->image;
    vector<uchar> t_vecImg; //Mat 图片数据转换为vector<uchar>
    imencode(".jpg", t_m_img, t_vecImg, compression_params);
    string t_imgbase64 = base64_encode(t_vecImg.data(), t_vecImg.size()); //实现图片的base64编码
    //赋值为字符串
    data.t_data = t_imgbase64;

    data.pos_x = current_pose.pose.pose.position.x;
    data.pos_y = current_pose.pose.pose.position.y;
    data.pos_z = current_pose.pose.pose.position.z;
    data.qua_x = current_pose.pose.pose.orientation.x;
    data.qua_y = current_pose.pose.pose.orientation.y;
    data.qua_z = current_pose.pose.pose.orientation.z;
    data.qua_w = current_pose.pose.pose.orientation.w;

    // data.horizontal = yt_pose.pose.pose.position.x;
    // data.vertical = yt_pose.pose.pose.position.z;

    rgb_pub.publish(data);

    if (is_record)
    {
        count = ros::Time::now().sec;
        scount = ros::Time::now().nsec;
        //start current frame
        try
        {
            //save image
            visible_image_name = "1.jpg";
            // std::stringstream v_ss;
            // v_ss << visible_path << "/"
            //      << count << "_" << scount << ".jpg";
            // visible_image_name = v_ss.str();
            // cout << "visible_image_name" << visible_image_name << endl;
            // imwrite(visible_image_name, v_m_img, compression_params);

            std::stringstream t_ss;
            t_ss << thermal_path << "/"
                 << count << "_" << scount << ".jpg";
            thermal_image_name = t_ss.str();
            cout << "thermal_image_name " << thermal_image_name << endl;
            imwrite(thermal_image_name, t_m_img, compression_params);

            //ofstream map_points;
            //map_points.open("/home/li/capture/0919/record.txt");
            map_points << visible_image_name << " " << thermal_image_name << " " << data.pos_x << " " << data.pos_y << " " << data.pos_z << " " << data.qua_x << " " << data.qua_y << " " << data.qua_z << " " << data.qua_w << " " << data.horizontal << " " << data.vertical << std::endl;
            //map_points.close();

            std::cout
                << "record success +1 " << std::endl;
        }
        catch (runtime_error &ex)
        {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }
        is_record = false;
    }

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

void message_sync_ros_node::isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
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
