#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core/core.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int camera_id;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "videocapture");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("/visible/image_proc/compressed", 1);

    ros::param::get("/visible_rtsp/camera_id", camera_id);
    std::cout << "camera_id:" << camera_id << std::endl;

    ros::Rate loop_rate(5);

    int count = 0;
    //cv::VideoCapture v_capture("rtsp://admin:1234qwer@192.168.1.64:554/h264/ch1/main/av_stream");
    cv::VideoCapture v_capture(camera_id);

    cv_bridge::CvImagePtr frame = boost::make_shared<cv_bridge::CvImage>();
    frame->encoding = sensor_msgs::image_encodings::BGR8;
    cv::Mat resize;

    while (ros::ok())
    {
        if (v_capture.isOpened())
        {
            //std::cout << " count： " << count << std::endl;
            v_capture >> frame->image; //流的转换
            if (frame->image.empty())
            {
                ROS_ERROR_STREAM("Failed to capture frame!");
                continue;
            }
            //打成ROS数据包
            frame->header.stamp = ros::Time::now();
            pub_image.publish(frame->toImageMsg());

            cv::waitKey(3); //opencv刷新图像 3ms
            ros::spinOnce();

            // cv::resize(frame->image, resize, cv::Size(1280, 720));

            // cv_bridge::CvImage resizeRos;
            // resizeRos.encoding = "bgr8";
            // resizeRos.image = resize;

            // sensor_msgs::ImagePtr imagePtr = resizeRos.toImageMsg();
            // imagePtr->width = 1280;
            // imagePtr->height = 720;
            // imagePtr->header.stamp = ros::Time::now();
            // pub_image.publish(imagePtr);
            //打成ROS数据包

            // cv::waitKey(3); //opencv刷新图像 3ms
            // ros::spinOnce();
        }
        else
        {
            //断开重连
            std::cout << "断开重连...　 " << std::endl;
            v_capture.release();
            sleep(3);
            //cv::VideoCapture v_capture("rtsp://admin:1234qwer@192.168.1.64:554/h264/ch1/main/av_stream");
            cv::VideoCapture v_capture(0);
            sleep(1);
        }
    }

    v_capture.release();
    return 0;
}