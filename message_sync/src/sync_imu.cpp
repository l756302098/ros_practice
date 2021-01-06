//sensor_msgs/Imu 

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace cv;
using namespace Eigen;

static void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
if (fabs(sinp) >= 1)
pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
else
pitch = asin(sinp);

// yaw (z-axis rotation)
double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}
static float toAngle(double& r)
{
    return 180 * r / M_PI;
}
class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Subscriber init_sub,yt_sub,imu_sub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    std::string yt_topic;
    std::string imu_topic;
    std::string root_path;
    bool is_record,is_init;
    ofstream map_points;
    stringstream gss;
    float init_yt_x,init_yt_y,init_yt_z,init_imu_x,init_imu_y,init_imu_z;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    void callback(const nav_msgs::Odometry::ConstPtr &yt_data,const sensor_msgs::Imu::ConstPtr &imu_data);
    void update();
    void initCB(const std_msgs::Bool::ConstPtr &msg);
    void ytCB(const nav_msgs::Odometry::ConstPtr &yt_data);
    void imuCB(const sensor_msgs::Imu::ConstPtr &imu_data);
    void isDirectory(string path);
};

message_sync_ros_node::message_sync_ros_node()
{
    is_record = false;
    is_init = false;
    //map_points.open("/home/li/capture/0919/record.txt", ios_base::app);
    ros::param::get("/sync_imu/yt_topic", yt_topic);
    ros::param::get("/sync_imu/imu_topic", imu_topic);
    ros::param::get("/sync_imu/root_path", root_path);
    std::cout << "yt_topic:" << yt_topic << std::endl;
    std::cout << "imu_topic:" << imu_topic << std::endl;
    std::cout << "root_path:" << root_path << std::endl;

    isDirectory(root_path);
    string record_path = root_path + "/record.txt";
    map_points.open(record_path, ios_base::app);
    init_sub = node_.subscribe<std_msgs::Bool>("sync_imu/init", 1, &message_sync_ros_node::initCB, this);
    yt_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, yt_topic, 1);
    imu_sub_ = new message_filters::Subscriber<sensor_msgs::Imu>(node_, imu_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *yt_sub_, *imu_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2));

    //sub
    yt_sub = node_.subscribe<nav_msgs::Odometry>(yt_topic, 1, &message_sync_ros_node::ytCB, this);
    imu_sub = node_.subscribe<sensor_msgs::Imu>(imu_topic, 1, &message_sync_ros_node::imuCB, this);
}

void message_sync_ros_node::initCB(const std_msgs::Bool::ConstPtr &msg){
    std::cout << "initCB " << std::endl;
     is_init = true;
}

message_sync_ros_node::~message_sync_ros_node()
{
    map_points.close();
}

void message_sync_ros_node::update()
{
   std::cout << "update " << std::endl;
}

void message_sync_ros_node::isDirectory(string path)
{
    //判断根目录下是否存在子目录，不存在创建
    if (!boost::filesystem::exists(path))
    {
        boost::filesystem::create_directory(path);
    }
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &yt_data,const sensor_msgs::Imu::ConstPtr &imu_data)
{
    std::cout << "sync message" << std::endl;
    // nav_msgs::Odometry current_pose;
    // current_pose = *odom_data;
    float angle_x = yt_data->pose.pose.position.x;
    float angle_y = yt_data->pose.pose.position.y;
    float angle_z = yt_data->pose.pose.position.z;
    //std::cout << "angle_x:" << angle_x<< " angle_y:" << angle_y<< " angle_z:" << angle_z << std::endl;
    //
    //float imu_x = imu_data->orientation.x;
    geometry_msgs::Quaternion qua = imu_data->orientation;
    Eigen::Quaterniond q(qua.w, qua.x, qua.y, qua.z);
    double roll,pitch,yaw;
    toEulerAngle(q,roll,pitch,yaw);
    //std::cout << "roll:" << roll<< " pitch:" << pitch<< " yaw:" << yaw << std::endl;
    if(is_init){
        std::cout << "init:" << std::endl;
        init_yt_x = angle_x;
        init_yt_y = angle_y;
        init_yt_z = angle_z;
        init_imu_x = toAngle(roll);
        init_imu_y = toAngle(pitch);
        init_imu_z = toAngle(yaw);
        is_init = false;
    }
    float dx = abs(angle_x - init_yt_x)/100;
    float dy = abs(angle_y - init_yt_y)/100;
    float dz = abs(angle_z - init_yt_z)/100;
    std::cout << "ptz dx:" << dx<< " dy:" << dy << " dz:" << dz << std::endl;
    map_points << "ptz dx:" << dx<< " dy:" << dy << " dz:" << dz << std::endl;
    dx = abs(toAngle(roll) - init_imu_x);
    dy = abs(toAngle(pitch) - init_imu_y);
    dz = abs(toAngle(yaw) - init_imu_z);
    //std::cout << "roll:" << toAngle(roll)<< " pitch:" << toAngle(pitch) << " yaw:" << toAngle(yaw) << std::endl;
    std::cout << "imu dx:" << dx<< " dy:" << dy << " dz:" << dz << std::endl;
    map_points << "imu dx:" << dx<< " dy:" << dy << " dz:" << dz << std::endl;
}

void message_sync_ros_node::ytCB(const nav_msgs::Odometry::ConstPtr &yt_data)
{
    float angle_x = abs(yt_data->pose.pose.position.x)/100;
    float angle_y = abs(yt_data->pose.pose.position.y)/100;
    float angle_z = abs(yt_data->pose.pose.position.z)/100; 
    std::cout << "PTZ data receive time:" << yt_data->header.stamp << " angle_x:" << angle_x<< " angle_y:" << angle_y << " angle_z:" << angle_z << std::endl;
    map_points << "PTZ data receive time:" << yt_data->header.stamp << " angle_x:" << angle_x<< " angle_y:" << angle_y << " angle_z:" << angle_z << std::endl;
}

void message_sync_ros_node::imuCB(const sensor_msgs::Imu::ConstPtr &imu_data)
{
    geometry_msgs::Quaternion qua = imu_data->orientation;
    Eigen::Quaterniond q(qua.w, qua.x, qua.y, qua.z);
    double roll,pitch,yaw;
    toEulerAngle(q,roll,pitch,yaw);
    if(is_init){
        std::cout << "init:" << std::endl;
        init_imu_x = toAngle(roll);
        init_imu_y = toAngle(pitch);
        init_imu_z = toAngle(yaw);
        is_init = false;
    }
    float dx = abs(toAngle(roll) - init_imu_x);
    float dy = abs(toAngle(pitch) - init_imu_y);
    float dz = abs(toAngle(yaw) - init_imu_z);
    std::cout << "IMU data receive time:" << imu_data->header.stamp << " roll:" << dx<< " pitch:" << dy << " yaw:" << dz << std::endl;
    map_points << "IMU data receive time:" << imu_data->header.stamp << " roll:" << dx<< " pitch:" << dy << " yaw:" << dz << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    message_sync_ros_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}