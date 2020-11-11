#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/PoseStamped.h>

class PoseTransform
{
public:
    PoseTransform();
    void update();

private:
    ros::Publisher pub;
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
};


PoseTransform::PoseTransform() : tf2_(buffer_)
{
    ros::NodeHandle nh("~");
    pub = nh.advertise<geometry_msgs::PoseStamped>("/cart/pose", 1, true);
}

void PoseTransform::update()
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = buffer_.lookupTransform("map","laser_link",
                                            ros::Time(0));
        //ROS_INFO("get message");
        geometry_msgs::PoseStamped lidar_pose;
        lidar_pose.header.frame_id = "map";
        lidar_pose.header.stamp = ros::Time::now();
        lidar_pose.pose.position.x = transform.transform.translation.x;
        lidar_pose.pose.position.y = transform.transform.translation.y;
        lidar_pose.pose.position.z = transform.transform.translation.z;
        lidar_pose.pose.orientation.x = transform.transform.rotation.x;
        lidar_pose.pose.orientation.y = transform.transform.rotation.y;
        lidar_pose.pose.orientation.z = transform.transform.rotation.z;
        lidar_pose.pose.orientation.w = transform.transform.rotation.w;
        pub.publish(lidar_pose);
    }
    catch (const std::exception &e)
    {
        ROS_INFO("transform error:%s",e.what());
        std::cerr << e.what() << '\n';
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cart_transform_node");
    PoseTransform pt;
    ros::Rate rate(20);

    while (ros::ok())
    {
        pt.update();
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("node exit!");
    return 0;
}