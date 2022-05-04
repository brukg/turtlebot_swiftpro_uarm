#include <ros/ros.h>

#include "swift_uarm/SQNCR.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_sequencer");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    SQNCR SQNCR(node, private_nh); //instance of CNTRL class

    ros::Rate loop_rate(10);


    ros::spin();

    return 0;
}