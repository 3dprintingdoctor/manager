#include "manager_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager_node");
    ros::NodeHandle nh_;

    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
       loop_rate.sleep();
    }
    return 0;
}


