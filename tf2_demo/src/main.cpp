#include "ros/ros.h"
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <geometry_msgs/TransformStamped.h>

using namespace Eigen;

std::string target_frame_,
    raw_frame_;

Eigen::Quaterniond enler2quration(Eigen::Vector3d eulerAngle)
{
    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2_listener");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    nh.param<std::string>("/tf2_demo/target_frame", target_frame_, "");
    nh.param<std::string>("/tf2_demo/raw_frame", raw_frame_, "");

    ros::Rate rate(5);
    while (ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform(target_frame_, raw_frame_,
                                                        ros::Time(0));

            //四元素->旋转矩阵
            Eigen::Quaterniond q2 = Eigen::Quaterniond(1, 0, 0, 0); //(w,x,y,z)
            std::cout << "q2:"
                      << q2.toRotationMatrix() << std::endl;

            Eigen::Vector3d eulerAngle = q2.matrix().eulerAngles(2, 1, 0);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}