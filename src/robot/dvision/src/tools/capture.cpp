// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/camera.hpp"
#include <opencv2/opencv.hpp>
#include <csignal>

void
signalHandler(int sig)
{
    ROS_WARN("Trying to exit!");
    std::terminate();
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh("~");
    signal(SIGINT, signalHandler);

    dvision::CameraSettings s(&nh);
    dvision::Camera c(s);

    std::cout << "press c to capture" << std::endl;
    while (ros::ok()) {
        auto frame = c.capture();
        auto& mat = frame.getBGR();
        cv::namedWindow("capture", CV_WINDOW_NORMAL);
        cv::imshow("capture", mat);
        char key = (char)cv::waitKey(1);
        if (key == 'c')
            frame.save("./");
    }
}
