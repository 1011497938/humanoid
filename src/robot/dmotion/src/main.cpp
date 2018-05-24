#include "dmotion/dmotion.hpp"
#include "dmotion/BehaviorRecv.hpp"
#include <ros/ros.h>
#include <signal.h>

using namespace dmotion;

DMotion* d = nullptr;
BehaviorRecv* b = nullptr;

void signalHandler(int sig) {
    ROS_WARN("Trying to exit!");
//    if (d != nullptr) {
//        d->attemptShutdown();
//        b->attemptShutdown();
//    }

    std::terminate();
}


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dmotion_node");
    ros::NodeHandle nh("~");

    signal(SIGINT, signalHandler);

    d = new DMotion(&nh);
    b = new BehaviorRecv(&nh, d);

    d->spin();
    b->spin();

    d->join();
    b->join();
}
