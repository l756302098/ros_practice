#include <yd_obstacle_avoid/intelligent_plan.h>

using namespace yd_obstacle_avoid;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intelligent_plan_node");

    intelligent_plan ip;

    //ros::spin();
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