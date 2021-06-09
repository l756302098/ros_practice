#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "base64.h"

//#include <unistd.h>
#include <stdio.h>
#include <termios.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

#include "cv.h"
#include "highgui.h"

using namespace std;
using namespace cv;

class save_image_node
{
private:
    ros::NodeHandle node_;
    ros::Subscriber sub;
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
    save_image_node();
    ~save_image_node();
    //void keyCallback(const keyboard::Key::ConstPtr &key_data);
    void tick(const ros::TimerEvent &event);
    void callback(const sensor_msgs::ImageConstPtr& msg);void update();
    void isDirectory(string path);
};
void save_image_node::tick(const ros::TimerEvent &event){
   counter = 0;
}
save_image_node::save_image_node()
{
    is_record = false;
    root_path = "/home/li/";
    sub = node_.subscribe("/fixed/infrared/raw", 1, &save_image_node::callback,this);
}

save_image_node::~save_image_node()
{
}

void save_image_node::update()
{
 
}

void save_image_node::isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
}

void save_image_node::callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    int height =  msg->height;
    int width =  msg->width;
    std::cout << "width:" << width << " height:" << height << std::endl;
    //cv::Mat Ma;
    vector<uchar> t_array = msg->data;
    vector<uchar> i_array;
    float f;
    //set data
    for (int i = 0; i < height * width * 4;)
    {
        /* code */
        float* t = (float*)&t_array[i];
        f = *t;
        //std::cout << " " << f ;
        i_array.push_back(uchar(f)+20);
        i+=4;
    }
    //std::cout << std::endl;
    Mat mat(width,height,CV_8UC1);
    for(int i = 0 ; i < mat.rows ; i ++) {
        for(int j = 0 ; j < mat.cols ; j ++){
            uchar &a = mat.at<uchar>(i,j);
            a = i_array[i*width+j];
        }
    }
    // Mat im_color;
    // applyColorMap(mat, im_color, COLORMAP_JET);
    cv::imshow("Display window",mat);
    cv::waitKey(5);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    save_image_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(20);
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
