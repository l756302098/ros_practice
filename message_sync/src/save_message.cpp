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

class save_message_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    ros::Subscriber key_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image> slamSyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> *visible_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *thermal_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
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
    std::atomic<int> counter;
    ros::Timer timer;

public:
    save_message_node();
    ~save_message_node();
    //void keyCallback(const keyboard::Key::ConstPtr &key_data);
    void tick(const ros::TimerEvent &event);
    void callback(const nav_msgs::Odometry::ConstPtr &pt_data, const sensor_msgs::ImageConstPtr &visible_image, const sensor_msgs::ImageConstPtr &thermal_image);
    void update();
    void isDirectory(string path);
};
void save_message_node::tick(const ros::TimerEvent &event){
   counter = 0;
}
save_message_node::save_message_node()
{
    is_record = false;
    //map_points.open("/home/li/capture/0919/record.txt", ios_base::app);
    ros::param::get("/message_sync/visible_topic", visible_topic);
    ros::param::get("/message_sync/thermal_topic", thermal_topic);
    ros::param::get("/message_sync/yt_topic", yt_topic);
    ros::param::get("/message_sync/root_path", root_path);
    ros::param::get("/message_sync/record", is_record);
    std::cout << "visible_topic:" << visible_topic << std::endl;
    std::cout << "thermal_topic:" << thermal_topic << std::endl;
    std::cout << "yt_topic:" << yt_topic << std::endl;
    std::cout << "root_path:" << root_path << std::endl;
    std::cout << "is_record:" << is_record << std::endl;

    isDirectory(root_path);
    visible_path = root_path + "/visible";
    isDirectory(visible_path);
    thermal_path = root_path + "/thermal";
    isDirectory(thermal_path);
    string record_path = root_path + "/record.csv";
    map_points.open(record_path, ios_base::app);
    timer = node_.createTimer(ros::Duration(1.0), &save_message_node::tick, this, false);
    //key_sub = node_.subscribe<keyboard::Key>("/keyboard/keydown", 1, &save_message_node::keyCallback, this);
    rgb_pub = node_.advertise<message_sync::pointcloud_color2>("/yd/pointcloud/vt", 1);
    visible_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_, visible_topic, 1);
    thermal_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_, thermal_topic, 1);
    yt_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, yt_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *yt_sub_, *visible_sub_, *thermal_sub_);
    sync_->registerCallback(boost::bind(&save_message_node::callback, this, _1, _2, _3));
}

save_message_node::~save_message_node()
{
    map_points.close();
}

void save_message_node::update()
{
 
}

void save_message_node::isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
}

void save_message_node::callback(const nav_msgs::Odometry::ConstPtr &pt_data, const sensor_msgs::ImageConstPtr &visible_image, const sensor_msgs::ImageConstPtr &thermal_image)
{
    //is_record = true;
    counter++;
    sensor_msgs::Image t_img;
    t_img = *thermal_image;

    sensor_msgs::Image v_img;
    v_img = *visible_image;

    nav_msgs::Odometry yt_pose;
    yt_pose = *pt_data;
    
    if (is_record)
    {
        count = ros::Time::now().sec;
        scount = ros::Time::now().nsec;
        std::string name = std::to_string(count)+"_"+std::to_string(scount);
        //start current frame
        try
        {
            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);

            cv_bridge::CvImagePtr v_cv_ptr = cv_bridge::toCvCopy(v_img, sensor_msgs::image_encodings::BGR8);
            cv::Mat v_m_img = v_cv_ptr->image;
            std::string v_path = "visible/" +  name + ".jpg";
            imwrite(root_path +"/"+v_path, v_m_img, compression_params);

            std::string t_path = "thermal/" +  name + ".txt";

            vector<uchar> t_array = t_img.data;
            std::cout << "t_array.size:" << t_array.size() << std::endl;
            // std::stringstream t_ss;
            // for (int i = 0; i < t_array.size(); i++)
            // {
            //     /* code */
            //     t_ss << t_array[i] << ",";
            // }
            // ofstream t_points;
            // t_points.open(root_path +"/"+t_path, ios_base::app);
            // t_points << t_ss.str()  << std::endl;
            // t_points.close();
            std::string t_imgbase64 = base64_encode(t_array.data(), t_array.size());
            ofstream t_points;
            t_points.open(root_path +"/"+t_path, ios_base::app);
            t_points << t_imgbase64  << std::endl;
            t_points.close();

            map_points << v_path << ","<< t_path << "," << yt_pose.pose.pose.position.x << "," << yt_pose.pose.pose.position.z << std::endl;

            std::cout
                << "record success +1 " << std::endl;
        }
        catch (runtime_error &ex)
        {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }
    }
    std::cout
        << "callback " << counter << std::endl;
        
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    save_message_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(20);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
