#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "message_sync/pointcloud_color.h"
#include "message_sync/pointcloud_color2.h"
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
using namespace cv;

class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    ros::Subscriber key_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CompressedImage, nav_msgs::Odometry> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *visible_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *thermal_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    std::string odom_topic;
    std::string visible_topic;
    std::string thermal_topic;
    std::string yt_topic;
    std::string root_path;
    std::string visible_path;
    std::string thermal_path;
    string visible_image_name;
    string thermal_image_name;
    int count, scount;
    bool is_record;
    ofstream map_points;
    stringstream gss;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    //void keyCallback(const keyboard::Key::ConstPtr &key_data);
    void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::ImageConstPtr &visible_image, const sensor_msgs::CompressedImage::ConstPtr &thermal_image, const nav_msgs::Odometry::ConstPtr &yt_data);
    void update();
    void isDirectory(string path);
};

message_sync_ros_node::message_sync_ros_node()
{
    is_record = false;
    //map_points.open("/home/li/capture/0919/record.txt", ios_base::app);
    ros::param::get("/message_sync/odom_topic", odom_topic);
    ros::param::get("/message_sync/visible_topic", visible_topic);
    ros::param::get("/message_sync/thermal_topic", thermal_topic);
    ros::param::get("/message_sync/yt_topic", yt_topic);
    ros::param::get("/message_sync/root_path", root_path);
    std::cout << "odom_topic:" << odom_topic << std::endl;
    std::cout << "visible_topic:" << visible_topic << std::endl;
    std::cout << "thermal_topic:" << thermal_topic << std::endl;
    std::cout << "yt_topic:" << yt_topic << std::endl;
    std::cout << "root_path:" << root_path << std::endl;

    isDirectory(root_path);
    visible_path = root_path + "/visible";
    isDirectory(visible_path);
    thermal_path = root_path + "/thermal";
    isDirectory(thermal_path);
    string record_path = root_path + "/record.txt";
    map_points.open(record_path, ios_base::app);

    //key_sub = node_.subscribe<keyboard::Key>("/keyboard/keydown", 1, &message_sync_ros_node::keyCallback, this);
    rgb_pub = node_.advertise<message_sync::pointcloud_color2>("/yd/pointcloud/vt", 1);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic, 1);
    visible_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_, visible_topic, 1);
    thermal_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, thermal_topic, 1);
    yt_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, yt_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *odom_sub_, *visible_sub_, *thermal_sub_, *yt_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2, _3, _4));
}

message_sync_ros_node::~message_sync_ros_node()
{
    map_points.close();
}

// void message_sync_ros_node::keyCallback(const keyboard::Key::ConstPtr &key_data)
// {
//     keyboard::Key key;
//     key = *key_data;
//     if (key.code == 99)
//     {
//         cout << "record +1" << endl;
//         is_record = true;
//     }
//     cout << "keyCallback" << key.code << endl;
// }

void message_sync_ros_node::update()
{
    // message_sync::pointcloud_color data;
    // data.v_data = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    // data.t_data = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12};

    // data.pos_x = 1;
    // data.pos_y = 2;
    // data.pos_z = 3;
    // data.qua_x = 1;
    // data.qua_y = 2;
    // data.qua_z = 3;
    // data.qua_w = 0;

    // rgb_pub.publish(data);
    // std::cout
    //     << "callback +1 " << std::endl;
}

void message_sync_ros_node::isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::ImageConstPtr &visible_image, const sensor_msgs::CompressedImage::ConstPtr &thermal_image, const nav_msgs::Odometry::ConstPtr &yt_data)
{
    //is_record = true;
    nav_msgs::Odometry current_pose;
    current_pose = *odom_data;

    sensor_msgs::CompressedImage t_img;
    t_img = *thermal_image;

    nav_msgs::Odometry yt_pose;
    yt_pose = *yt_data;

    message_sync::pointcloud_color2 data;
    data.v_format = ".jpg";
    //data.v_data = v_img.data;
    //cv_bridge::CvImagePtr v_cv_ptr = cv_bridge::toCvCopy(v_img, sensor_msgs::image_encodings::BGR8);
    //std::cout << "encoding:" << visible_image->encoding << std::endl;
    cv_bridge::CvImagePtr v_cv_ptr = cv_bridge::toCvCopy(visible_image, visible_image->encoding);

    //std::cout << "rows" << v_cv_ptr->image.rows << "cols" << v_cv_ptr->image.cols << std::endl;
    int rows = v_cv_ptr->image.rows;
    int cols = v_cv_ptr->image.cols;
    //clear
    gss.str("");

    for (int i = 0; i < rows; ++i)
    {
        const uint16_t *praw = v_cv_ptr->image.ptr<uint16_t>(i);
        for (int j = 0; j < cols; ++j)
        {
            uint16_t point = praw[j];
            gss << point;
            gss << ",";
        }
    }

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    //赋值为字符串
    string gray_str = gss.str();
    gray_str = gray_str.substr(0, gray_str.length() - 1);
    data.gray_data = gray_str;

    data.t_format = ".jpg";
    //data.t_data = t_img.data;
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

    data.horizontal = yt_pose.pose.pose.position.x;
    data.vertical = yt_pose.pose.pose.position.z;

    rgb_pub.publish(data);

    if (is_record)
    {
        count = ros::Time::now().sec;
        scount = ros::Time::now().nsec;
        //start current frame
        try
        {
            //save image
            std::stringstream v_ss;
            v_ss << visible_path << "/"
                 << count << "_" << scount << ".jpg";
            visible_image_name = v_ss.str();
            cout << "visible_image_name" << visible_image_name << endl;
            //imwrite(visible_image_name, *mergeImage, compression_params);

            std::stringstream t_ss;
            t_ss << thermal_path << "/"
                 << count << "_" << scount << ".jpg";
            thermal_image_name = t_ss.str();
            cout << "thermal_image_name " << thermal_image_name << endl;
            imwrite(thermal_image_name, t_m_img, compression_params);

            //ofstream map_points;
            //map_points.open("/home/li/capture/0919/record.txt");
            map_points << "1.jpg"
                       << " " << thermal_image_name << " " << data.pos_x << " " << data.pos_y << " " << data.pos_z << " " << data.qua_x << " " << data.qua_y << " " << data.qua_z << " " << data.qua_w << " " << data.horizontal << " " << data.vertical << " " << visible_image_name << std::endl;
            //map_points.close();
            //usleep(3);
            std::cout
                << "record success +1 " << std::endl;
        }
        catch (runtime_error &ex)
        {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }

        //is_record = false;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    message_sync_ros_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(10);

    while (ros::ok())
    {
        // int seconds = ros::Time::now().sec;
        // int nseconds = ros::Time::now().nsec;
        // std::cout << "seconds:" << seconds << std::endl;
        // std::cout << "nseconds:" << nseconds << std::endl;
        //node.update();,
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
