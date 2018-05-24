// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"
#include <signal.h>

void
signalHandler(int sig)
{
    ROS_WARN("Trying to exit!");
    std::terminate();
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dvision_5");
    ros::NodeHandle n("~");

    signal(SIGINT, signalHandler);
    dvision::DVision v;
    v.Start();
    v.Join();
}
