#include <ros/ros.h>

#include "swift_uarm/CNTRL.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    CNTRL CNTRL(node, private_nh); //instance of CNTRL class

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}