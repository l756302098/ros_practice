#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class PoseDrawer
{
public:
    PoseDrawer();
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);

private:
    ros::Publisher center_pub;
    ros::Subscriber robot_pose_sub;
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    ros::NodeHandle n_;
};

PoseDrawer::PoseDrawer() : tf2_(buffer_), target_frame_("turtle1")
{
    robot_pose_sub = n_.subscribe("/robot_pose", 1, &PoseDrawer::pose_callback,this);
    center_pub = n_.advertise<geometry_msgs::PoseStamped>("/yida/robot/center", 1, true);
}

void PoseDrawer::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    geometry_msgs::PoseStamped lidar_pose;
    lidar_pose.header.frame_id = "lidar_pose";
    ros::Time current_time = ros::Time::now();
    lidar_pose.header.stamp = current_time;
    lidar_pose.pose.position.x = pose_msg->pose.pose.position.x;
    lidar_pose.pose.position.y = pose_msg->pose.pose.position.y;
    lidar_pose.pose.position.z = pose_msg->pose.pose.position.z;
    lidar_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    lidar_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    lidar_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    lidar_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

    geometry_msgs::PoseStamped center_pose;
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = buffer_.lookupTransform("lidar_pose","center_pose",
                                            ros::Time(0));
        //buffer_.transform(lidar_pose, center_pose, "center_pose");
        tf2::doTransform(lidar_pose, center_pose, transform);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    center_pose.header.frame_id = "map";
    center_pose.header.stamp = current_time;
    std::cout << "pose:" << center_pose << std::endl;
    center_pub.publish(center_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_transfrom"); //Init ROS
    PoseDrawer pd;                        //Construct class
    ros::spin();                          // Run until interupted
    return 0;
};