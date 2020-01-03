#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

class ImageConverterCV
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int count;
    int total;

public:
    ImageConverterCV()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        //image_sub_ = it_.subscribe("/thermal/image_raw", 1,
        //&ImageConverterCV::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        //cv::namedWindow(OPENCV_WINDOW);
        readImage();
    }

    ~ImageConverterCV()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

    void PrintMat(Mat A)
    {
        for (int i = 0; i < A.rows; i++)
        {
            for (int j = 0; j < A.cols; j++)
            {
                uint8_t data = A.at<uint8_t>(i, j);
                if (data <= 0)
                {
                    total++;
                    printf("%i,", total);
                }
            }
            printf("\n");
        }
        printf("\n");
    }

    void readImage()
    {
        std::string image_name = "/home/li/capture/1222-1/gray/1577084565_279929537.jpg";
        cv::Mat color_image;
        color_image = cv::imread(image_name, cv::IMREAD_COLOR);

        std::vector<cv::Mat> v_channel;
        cv::Mat blue_channel, green_channel, red_channel;
        cv::split(color_image, v_channel);
        red_channel = v_channel.at(0);
        green_channel = v_channel.at(1);
        blue_channel = v_channel.at(2);

        std::cout << "rows:" << red_channel.rows << " cols:" << red_channel.cols << std::endl;
        PrintMat(blue_channel);
        std::cout << "==========================" << std::endl;
        //PrintMat(green_channel);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverterCV ic;
    ros::spin();
    return 0;
}