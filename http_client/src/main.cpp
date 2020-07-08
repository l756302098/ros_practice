#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/CompressedImage.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <cpprest/containerstream.h>
#include "multipart_parser.h"

#include <http_client/H264Decoder.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <curl/curl.h>
#include <string>

#include <base64.h>


using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams
using namespace std;
using namespace boost;

struct RquestData
{
    std::string name;
    geometry_msgs::Pose pose;
};

std::vector<RquestData> data;
geometry_msgs::Pose robot_pose;
cv::Mat g_result_pic;
H264Decoder h264_decoder;
bool is_com;
std::string root_path,image_path;
std::string image_name;
ofstream os_text;

//回调函数  得到响应内容
int write_data(void* buffer, int size, int nmemb, void* userp){
    std::string * str = dynamic_cast<std::string *>((std::string *)userp);
    str->append((char *)buffer, size * nmemb);
    return nmemb;
}

int upload(string url, string &body,  string* response,string name)
{
    CURL *curl;
    CURLcode ret;
    curl = curl_easy_init();
    struct curl_httppost* post = NULL;
    struct curl_httppost* last = NULL;
    if (curl)
    {
        //const string image_name = image_path +name;
        const string image_name = "/home/li/capture/051401/image/1589431870_790853906.jpg";
        curl_easy_setopt(curl, CURLOPT_URL, (char *)url.c_str());           //指定url
        //curl_formadd(&post, &last, CURLFORM_PTRNAME, "path", CURLFORM_PTRCONTENTS, "device_cover", CURLFORM_END);//form-data key(path) 和 value(device_cover)
        //curl_formadd(&post, &last, CURLFORM_PTRNAME,  "file", CURLFORM_FILE, image_name.c_str(),CURLFORM_FILENAME, "1589431870.jpg", CURLFORM_END);// form-data key(file) "./test.jpg"为文件路径  "hello.jpg" 为文件上传时文件名
        curl_formadd(&post, &last, CURLFORM_COPYNAME,  "file", CURLFORM_FILE, image_name.c_str(),CURLFORM_FILENAME, "1589431870.jpg", CURLFORM_END);// form-data key(file) "./test.jpg"为文件路径  "hello.jpg" 为文件上传时文件名
        
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

void test2(){
    std::ifstream ifile("/home/li/capture/051401/image/1589431870_790853906.jpg", std::ifstream::binary);
    std::vector<char> data( ( std::istreambuf_iterator<char>( ifile ) ), std::istreambuf_iterator<char>() );
    std::string base_str = base64_encode((unsigned char*)&data[0], (unsigned int)data.size(),false);
    ifile.close();

    CURL *hnd = curl_easy_init();
    if(hnd)
    {
    curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(hnd, CURLOPT_URL, "http://suerey.oicp.net:25948/upload.php");
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "postman-token:99acbcae-6a84-d8b2-4035-c40c32a44825");
    headers = curl_slist_append(headers, "cache-control: no-cache");
    headers = curl_slist_append(headers, "content-type:multipart/form-data; boundary=----WebKitFormBoundary7MA4YWxkTrZu0gW");
    curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(hnd, CURLOPT_POSTFIELDS,"------WebKitFormBoundary7MA4YWxkTrZu0gW\r\nContent-Disposition: form-data;name=\"file\"; filename=\"1589431870_790853906.jpg\"\r\nContent-Type:image/jpeg\r\n\r\n"+base_str+"\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW--");
    CURLcode ret = curl_easy_perform(hnd);
    long http_code = 0;
    curl_easy_getinfo (hnd, CURLINFO_RESPONSE_CODE, &http_code);
    if (ret == CURLE_OK)
    {
        cout << "Request sent" << endl;
        if (http_code == 200)
        {
        cout << "File sent and processed" << endl;
        }
        else
        {
            cout << "file upload failed, check request" << endl;
            /* always cleanup */
            curl_easy_cleanup(hnd);
        }
   }
}
}

void postImage(RquestData &data){
    std::string body;
	std::string response;

    std::cout << "current pos:" << data.pose << std::endl;
    int start_time = ros::Time::now().sec;
    //int status_code = upload("http://suerey.oicp.net:25948/upload.php", body, &response,data.name);
    int status_code = upload("http://192.168.0.54/upload.php", body, &response,data.name);
	if (status_code != CURLcode::CURLE_OK) {
			std::cout << "error code:" << status_code  << std::endl;
	}
    int speed_time = ros::Time::now().sec - start_time;
    std::cout << "speed time:" << speed_time << std::endl;
   

    std::cout << body << std::endl;
	std::cout << response << std::endl;

    regex re("\\[.*\\]");
    cmatch what;

    if (regex_search(response.c_str(), what, re)) {

        cout << "match " << what.size() << endl;
        cout <<  what[0] << endl;


        os_text << data.name << " " << data.pose.position.x << "/" << data.pose.position.y << "/" 
        << data.pose.position.z << "/" << data.pose.orientation.x << "/" << data.pose.orientation.y 
        << "/" << data.pose.orientation.z << "/" << data.pose.orientation.w << " "<< what[0] << std::endl;

        // for (int i = 0; i < what.size(); i++) {
        //    // cout << i <<":" <<  what[i].second << endl; 
        //    //cout << "what[" << i << "]: " << what[i] << ", first: " << what[i].first << ", second: " << what[i].second << endl;
        // }
    } else {
        cout << "not match " << endl;

        os_text << data.name << " " << data.pose.position.x << "/" << data.pose.position.y << "/" 
        << data.pose.position.z << "/" << data.pose.orientation.x << "/" << data.pose.orientation.y 
        << "/" << data.pose.orientation.z << "/" << data.pose.orientation.w << " "<< "failed" << std::endl;
    }

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
             //set flag
            is_com = true;

            h264_decoder.decode(pBuffer, dwBufSize);
            g_result_pic = h264_decoder.getMat();

            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);

            //save image
            std::stringstream v_ss;
            v_ss << ros::Time::now().sec << "_" << ros::Time::now().nsec << ".jpg";
            image_name = v_ss.str();
            cout << "image_name:" << image_name << endl;
            //imwrite(visible_image_name, *mergeImage, compression_params);
            imwrite(image_path+image_name, g_result_pic);
            g_result_pic.release();
            RquestData dd;
            dd.name = image_name;
            dd.pose = robot_pose;
            data.push_back(dd);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
 }


void camera_callback(const sensor_msgs::CompressedImage::ConstPtr &msg)
 {
    ROS_INFO("get image");
    if(is_com) return;
    cv_bridge::CvImagePtr v_cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat v_m_img = v_cv_ptr->image;

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

            //save image
            std::stringstream v_ss;
            v_ss << ros::Time::now().sec << "_" << ros::Time::now().nsec << ".jpg";
            image_name = v_ss.str();
            cout << "image_name:" << image_name << endl;
            //imwrite(visible_image_name, *mergeImage, compression_params);
            imwrite(image_path+image_name, v_m_img);
            RquestData dd;
            dd.name = image_name;
            dd.pose = robot_pose;
            data.push_back(dd);
             //set flag
            is_com = true;
 }

void isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
}

void test(){
    string target("result[[-3.644144 23.933931 -9.407144]],[aaa]<>");
    //string target = "[[-3.644144 23.933931 -9.407144]]";
    //regex re("(\\[[^\\[]*\\])");
    //regex re("\\[(.*)\\]");
    regex re("\\[.*\\]");
    cmatch what;

    if (regex_search(target.c_str(), what, re)) {

        cout << "match " << what.size() << endl;

        for (int i = 0; i < what.size(); i++) {
           // cout << i <<":" <<  what[i].second << endl; 
           cout << "what[" << i << "]: " << what[i] << ", first: " << what[i].first << ", second: " << what[i].second << endl;
        }
    } else {
        cout << "not match " << endl;
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "http_client");
    ros::NodeHandle nh_;

    ros::param::get("/http_client/root_path", root_path);
    isDirectory(root_path);
    image_path = root_path + "/image/";
    isDirectory(image_path);
    std::cout <<"image_path:" << image_path <<  std::endl;

    string record_path = root_path + "/record.txt";
    os_text.open(record_path, ios_base::app);

    ros::Subscriber  robot_pose_sub = nh_.subscribe("/robot_pose", 1, &pose_callback);
    //ros::Subscriber  getstream_sub_ = nh_.subscribe("/yida/visible/image_proc", 1, &getstream_callback);
    ///camera1/compressed
    ros::Subscriber  getstream_sub_ = nh_.subscribe("camera1/compressed", 1, &camera_callback);
    
    ros::Publisher sim_pose = nh_.advertise<nav_msgs::Odometry>("/relocal/pose", 1, false);
    //RquestData rq;
    //postImage(rq);
    //test();
    test2();

    ROS_INFO("http_client node started...");
    ros::Rate rate(10); 

    while (ros::ok())
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";
        odom.pose.pose = robot_pose;
        sim_pose.publish(odom);

        if(data.size()>0){
            RquestData rd = data.front();
            //TODO: request pose from image
            postImage(rd);
            data.pop_back();
            //save result
            is_com = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    os_text.close();
    return 0;
}