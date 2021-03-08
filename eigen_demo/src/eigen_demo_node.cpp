/*
 * @Descripttion: 
 * @version: 
 * @Author: li
 * @Date: 2021-02-28 11:33:51
 * @LastEditors: li
 * @LastEditTime: 2021-03-01 17:57:49
 */
#include "ros/ros.h"
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include "eigen3/Eigen/Dense"
#include <opencv2/core.hpp>
#include "opencv2/core/eigen.hpp"
using namespace cv;
using namespace Eigen;
using namespace std;

geometry_msgs::PoseStamped c_pos,t_pos;

void ReadCalibrationFile(std::string _choose_file,cv::Mat &out_RT)
{
    std::cout << "_choose_file: " << _choose_file << std::endl;
    cv::FileStorage fs(_choose_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "Cannot open file calibration file" << _choose_file << std::endl;
    }
    else
    {
      fs["CameraExtrinsicMat"] >> out_RT;
    }
}

void readCameraParams(std::string& file){
    std::cout << "file:" << file << std::endl;
    cv::Mat color_lidar_exRT;
    std::string color_lidar_yamlfilepath;
    ReadCalibrationFile(file, color_lidar_exRT);

    Eigen::Matrix4d transform;
    cv::cv2eigen(color_lidar_exRT, transform);
    std::cout << "Matrix4d:"<< transform.matrix() << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> a3d_transform(transform);

    std::cout <<"translation:" <<a3d_transform.translation() << std::endl;
    std::cout <<"linear:" <<a3d_transform.linear() << std::endl;
    Eigen::Quaterniond qua;
    Eigen::Matrix3d rotation;
    rotation << a3d_transform.linear();
    qua = rotation;
}

void update(){
    /*
    Eigen::Matrix3d rotMatrix;
    Eigen::Vector3d vectorBefore(x, y, z)
    Eigen::Vector3d vectorAfter(a, b, c);
 
    rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();
    */
   //calc angle
    
}

void target_callback(const geometry_msgs::PoseStampedConstPtr &msg){
    t_pos.pose.position.x = msg->pose.position.x;
    t_pos.pose.position.y = msg->pose.position.y;
    t_pos.pose.position.z = msg->pose.position.z;
    Eigen::Quaterniond q(c_pos.pose.orientation.w, c_pos.pose.orientation.x, c_pos.pose.orientation.y, c_pos.pose.orientation.z);
    //forward
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d f = q * v;
    
    Eigen::Vector3d c(t_pos.pose.position.x-c_pos.pose.position.x, t_pos.pose.position.y-c_pos.pose.position.y, t_pos.pose.position.z-c_pos.pose.position.z);

    Eigen::Matrix3d rotMatrix;
    rotMatrix = Eigen::Quaterniond::FromTwoVectors(f, c).toRotationMatrix();
    std::cout << "matrix:" << rotMatrix.matrix() << std::endl;
    Eigen::Vector3d eulerAngle=rotMatrix.eulerAngles(0,1,2);
    std::cout << "roll:" << eulerAngle(0) << std::endl;
    std::cout << "pitch:" << eulerAngle(1) << std::endl;
    std::cout << "yaw:" << eulerAngle(2) << std::endl;
    std::cout << "----------------------" << std::endl;
    std::cout << "roll:" << eulerAngle(0)*180/M_PI << std::endl;
    std::cout << "pitch:" << eulerAngle(1)*180/M_PI << std::endl;
    std::cout << "yaw:" << eulerAngle(2)*180/M_PI << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_demo_node");
    ros::NodeHandle nh("~");
    std::string camera_file = nh.param<std::string>("camera_file", "");
    //readCameraParams(camera_file);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    ros::Subscriber sub = nh.subscribe("/target", 1, &target_callback);
    ROS_INFO("eigen_demo_node started...");
    ros::Rate rate(10);
    /*
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 2.4668250442e+01, 6.320669323e+00, -1.3322685606304879e+02;
    
    Eigen::Matrix3f rotation(3,3);
    rotation << -5.08698404e-01, 1.58019021e-01, -8.46319020e-01,
                8.60769331e-01, 1.13197252e-01, -4.96248573e-01,
                1.73842758e-02, -9.80926335e-01, -1.93601176e-01;
    transform.linear() = rotation;

    Eigen::Matrix3d rotation1(3,3);
    rotation1 << -5.08698404e-01, 1.58019021e-01, -8.46319020e-01,
                8.60769331e-01, 1.13197252e-01, -4.96248573e-01,
                1.73842758e-02, -9.80926335e-01, -1.93601176e-01;
    Eigen::Quaterniond qua;
    qua = rotation1;

    Eigen::Vector3d eulerAngle=rotation1.eulerAngles(2,1,0);
    std::cout << "roll:" << eulerAngle(0) << std::endl;
    std::cout << "pitch:" << eulerAngle(1) << std::endl;
    std::cout << "yaw:" << eulerAngle(2) << std::endl;
    std::cout << transform.matrix() << std::endl;
    std::cout << transform.translation() << std::endl;
    std::cout << transform.linear() << std::endl;
    */
    cv::Mat color_lidar_exRT;
    std::string color_lidar_yamlfilepath;
    ReadCalibrationFile(camera_file, color_lidar_exRT);

    Eigen::Matrix4d transform;
    cv::cv2eigen(color_lidar_exRT, transform);
    std::cout << "Matrix4d:"<< transform.matrix() << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> a3d_transform(transform);

    std::cout <<"translation:" <<a3d_transform.translation() << std::endl;
    std::cout <<"linear:" <<a3d_transform.linear() << std::endl;
    Eigen::Quaterniond qua;
    Eigen::Matrix3d rotation;
    rotation << a3d_transform.linear();
    qua = rotation;

    Eigen::Vector3d eulerAngle=rotation.eulerAngles(2,1,0);
    std::cout << "roll:" << eulerAngle(0) << std::endl;
    std::cout << "pitch:" << eulerAngle(1) << std::endl;
    std::cout << "yaw:" << eulerAngle(2) << std::endl;

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = a3d_transform.translation()[0];
    goal.pose.position.y = a3d_transform.translation()[1];
    goal.pose.position.z = a3d_transform.translation()[2];
    goal.pose.orientation.x = qua.x();
    goal.pose.orientation.y = qua.y();
    goal.pose.orientation.z = qua.z();
    goal.pose.orientation.w = qua.w();
    c_pos.pose.position = goal.pose.position;
    c_pos.pose.orientation = goal.pose.orientation;
    while (ros::ok())
    {
        update();
        pub.publish(goal);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
