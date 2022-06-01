#include <ros/ros.h>

#include "swift_uarm/ARUCO.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ARUCO ARUCO(node, private_nh); //instance of CNTRL class

    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}