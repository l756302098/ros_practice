#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/utils.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <signal.h>

class PoseDrawer
{
public:
    PoseDrawer();
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void transfrom(geometry_msgs::PoseStamped pose, geometry_msgs::TransformStamped transform);
    ros::NodeHandle n_;

private:
    ros::Publisher center_pub;
    ros::Subscriber robot_pose_sub;
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    
};

PoseDrawer::PoseDrawer() : tf2_(buffer_), target_frame_("turtle1")
{
    robot_pose_sub = n_.subscribe("/robot_pose", 1, &PoseDrawer::pose_callback,this);
    center_pub = n_.advertise<geometry_msgs::PoseStamped>("/yida/robot/center", 1, true);
}

void PoseDrawer::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    geometry_msgs::PoseStamped lidar_pose;
    lidar_pose.pose.position.x = pose_msg->pose.pose.position.x;
    lidar_pose.pose.position.y = pose_msg->pose.pose.position.y;
    lidar_pose.pose.position.z = pose_msg->pose.pose.position.z;
    lidar_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    lidar_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    lidar_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    lidar_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

    geometry_msgs::TransformStamped transform;
    try
    {
        transform = buffer_.lookupTransform("lidar_pose","center_pose",
                                            ros::Time(0));
        transfrom(lidar_pose, transform);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void PoseDrawer::transfrom(geometry_msgs::PoseStamped pose, geometry_msgs::TransformStamped transform)
{
    //1.p1 world position
    double p1x = pose.pose.position.x;
    double p1y = pose.pose.position.y;
    Eigen::Vector3d t1 = Eigen::Vector3d(p1x, p1y, 0);
    Eigen::Quaterniond q1(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    
    //2. t12 value
    double t12x = transform.transform.translation.x;
    double t12y = transform.transform.translation.y;
    Eigen::Vector3d t12 = Eigen::Vector3d(t12x, t12y, 0);
    Eigen::Quaterniond q12(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);

    //calc p2
    Eigen::Quaterniond q2 = q1 * q12;
    Eigen::Vector3d t2 = t1 + q1.toRotationMatrix() * t12;

    geometry_msgs::PoseStamped center_pose;
    center_pose.pose.position.x = t2(0);
    center_pose.pose.position.y = t2(1);
    center_pose.pose.position.z = pose.pose.position.z;

    center_pose.pose.orientation.x = q2.x();
    center_pose.pose.orientation.y = q2.y();
    center_pose.pose.orientation.z = q2.z();
    center_pose.pose.orientation.w = q2.w();

    center_pose.header.frame_id = "map";
    center_pose.header.stamp = ros::Time::now();
    std::cout << "center_pose:" << center_pose << std::endl;
    center_pub.publish(center_pose);
}

// void MySigintHandler(int sig)
// {
//     //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
//     ROS_INFO("shutting down!");
//     ros::shutdown();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_transfrom"); //Init ROS
    if (argc>1){
        std::cout << "argc" << argv[1] << std::endl;
    }

    PoseDrawer pd;
    bool is_sim = false;
    pd.n_.param<bool>("is_sim", is_sim, false);
    std::cout << "is_sim" << is_sim << std::endl;
    //signal(SIGINT, MySigintHandler);      //Construct class
    ros::spin();                          // Run until interupted
    ROS_INFO("node exits");
    return 0;
};