#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <cpprest/containerstream.h>
#include "multipart_parser.h"

#include <http_client/H264Decoder.h>

#include <base64.h>

#include <vector>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams
//using namespace std;



geometry_msgs::Pose robot_pose;
cv::Mat g_result_pic;
H264Decoder h264_decoder;
bool is_com;

void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    ROS_INFO("get pose");
    robot_pose.position.x = pose_msg->pose.pose.position.x;
    robot_pose.position.y = pose_msg->pose.pose.position.y;
    robot_pose.position.z = pose_msg->pose.pose.position.z;
    robot_pose.orientation.x = pose_msg->pose.pose.orientation.x;
    robot_pose.orientation.y = pose_msg->pose.pose.orientation.y;
    robot_pose.orientation.z = pose_msg->pose.pose.orientation.z;
    robot_pose.orientation.w = pose_msg->pose.pose.orientation.w;
}

void getstream_callback(const sensor_msgs::Image& msg)
 {
    ROS_INFO("get image");
    if(is_com) return;
    try
    {
        std::vector<unsigned char> vc;
        vc = msg.data;
        unsigned char* pBuffer = &vc.at(0);
        int dwBufSize = vc.size();
        if(pBuffer[4] == 0x67)
        {
            h264_decoder.decode(pBuffer, dwBufSize);
            g_result_pic = h264_decoder.getMat();
             //set flag
            is_com = true;
            //TODO: request pose from image

            is_com = false;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
 }

void postImage(){

    auto fileStream = std::make_shared<ostream>();

    // Open stream to output file.
    pplx::task<void> requestTask = fstream::open_ostream(U("results")).then([=](ostream outFile)
    {
        *fileStream = outFile;

        //Use MultipartParser to get the encoded body content and boundary
        MultipartParser parser;
        parser.AddParameter("failname","11");
        parser.AddFile("file", "/home/li/Downloads/2134551912.jpg");
        std::string boundary = parser.boundary();
        std::string body = parser.GenBodyContent();
        
        
        //std::cout << body << std::endl;

        //Set up http client and request
        http_request req;
        http_client client(U("http://suerey.oicp.net:25948/upload.php"));
        req.set_method(web::http::methods::POST);
        req.set_body(body, "multipart/form-data; boundary=" + boundary);

        //std::cout << req.relative_uri() << std::endl;
        std::cout << req.to_string() << std::endl;

        return client.request(req);
    })
    .then([=](pplx::task<http_response> response_task)
    {
        http_response response = response_task.get();
        return response.body().read_to_end(fileStream->streambuf());
    })
    .then([=](size_t)
    {
        return fileStream->close();
    });

    // Wait for all the outstanding I/O to complete and handle any exceptions
    try
    {
        requestTask.wait();
    }
    catch (const std::exception &e)
    {
        printf("Error exception:%s\n", e.what());
    }


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "http_client");
    ros::NodeHandle nh_;

    ros::Subscriber  robot_pose_sub = nh_.subscribe("/robot_pose", 1, &pose_callback);
    ros::Subscriber  getstream_sub_ = nh_.subscribe("/yida/visible/image_proc", 1, &getstream_callback);

    ROS_INFO("http_client node started...");
    ros::Rate rate(10);

    postImage();
    /* test http get 
    auto fileStream = std::make_shared<ostream>();
    
    // Open stream to output file.
    pplx::task<void> requestTask = fstream::open_ostream(U("results.html")).then([=](ostream outFile) {
         *fileStream = outFile;
        
         // Create http_client to send the request.
         http_client client(U("http://www.baidu.com/"));
        
         // Build request URI and start the request.
         uri_builder builder(U("/search"));
         builder.append_query(U("q"), U("cpprestsdk github"));
         return client.request(methods::GET,"");
    })
    
    // Handle response headers arriving.
    .then([=](http_response response) {
        printf("Received response status code:%u\n", response.status_code());
 
        // Write response body into the file.
        return response.body().read_to_end(fileStream->streambuf());
    })
    
    // Close the file stream.
    .then([=](size_t) {
        return fileStream->close();
    });
    
    // Wait for all the outstanding I/O to complete and handle any exceptions
    try {
        requestTask.wait();
    } catch (const std::exception &e) {
        printf("Error exception:%s\n", e.what());
    }
    */

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}