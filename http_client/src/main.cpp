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
#include <vector>

#include <curl/curl.h>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams
using namespace std;

struct RquestData
{
    std::string path;
    geometry_msgs::Pose pose;
};

std::vector<RquestData> data;
geometry_msgs::Pose robot_pose;
cv::Mat g_result_pic;
H264Decoder h264_decoder;
bool is_com;
std::string root_path,image_path;
std::string image_name;

//回调函数  得到响应内容
int write_data(void* buffer, int size, int nmemb, void* userp){
    std::string * str = dynamic_cast<std::string *>((std::string *)userp);
    str->append((char *)buffer, size * nmemb);
    return nmemb;
}

int upload(string url, string &body,  string* response,string &image_path)
{
    CURL *curl;
    CURLcode ret;
    curl = curl_easy_init();
    struct curl_httppost* post = NULL;
    struct curl_httppost* last = NULL;
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, (char *)url.c_str());           //指定url
        //curl_formadd(&post, &last, CURLFORM_PTRNAME, "path", CURLFORM_PTRCONTENTS, "device_cover", CURLFORM_END);//form-data key(path) 和 value(device_cover)
        curl_formadd(&post, &last, CURLFORM_PTRNAME,  "file", CURLFORM_FILE, image_path,CURLFORM_FILENAME, "hello.jpg", CURLFORM_END);// form-data key(file) "./test.jpg"为文件路径  "hello.jpg" 为文件上传时文件名
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, post);                     //构造post参数    
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);          //绑定相应
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)response);        //绑定响应内容的地址

        ret = curl_easy_perform(curl);                          //执行请求
        if(ret == 0){
            curl_easy_cleanup(curl);    
            return 0;  
        }
        else{
            return ret;
        }
    }
	else{
        return -1;
	}
}

void postImage(RquestData &data){
    std::string body;
	std::string response;

    std::cout << "current pos:" << data.pose << std::endl;
    int status_code = upload("http://suerey.oicp.net:25948/upload.php", body, &response,data.path);
	if (status_code != CURLcode::CURLE_OK) {
			std::cout << "error code:" << status_code  << std::endl;
	}
    std::cout << body << std::endl;
	std::cout << response << std::endl;
}

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

            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);

            //save image
            std::stringstream v_ss;
            v_ss << image_path << "/"
                 << ros::Time::now().sec << "_" << ros::Time::now().nsec << ".jpg";
            image_name = v_ss.str();
            cout << "image_name" << image_name << endl;
            //imwrite(visible_image_name, *mergeImage, compression_params);
            imwrite(image_name, g_result_pic, compression_params);
            RquestData dd;
            dd.path = image_name;
            dd.pose = robot_pose;
            data.push_back(dd);
             //set flag
            is_com = true;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
 }

void update(){
    if(data.size()>0){
        RquestData rd = data.front();
        //TODO: request pose from image
        postImage(rd);
        data.pop_back();
        //save result
        is_com = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "http_client");
    ros::NodeHandle nh_;

    ros::param::get("/http_client/root_path", root_path);
    image_path = root_path + "/image";
    std::cout <<"image_path:" << image_path <<  std::endl;
    ros::Subscriber  robot_pose_sub = nh_.subscribe("/robot_pose", 1, &pose_callback);
    ros::Subscriber  getstream_sub_ = nh_.subscribe("/yida/visible/image_proc", 1, &getstream_callback);

    ROS_INFO("http_client node started...");
    ros::Rate rate(10); 

    while (ros::ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}