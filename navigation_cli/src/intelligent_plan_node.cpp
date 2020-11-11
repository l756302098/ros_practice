#include <intelligent_plan.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_client_node");

    intelligent_plan ip;

    ros::Rate rate(10);

    while (ros::ok())
    {
        ip.update();
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("node exit!");
    return 0;
}