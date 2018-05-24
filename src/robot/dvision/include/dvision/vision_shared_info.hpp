#pragma once
#include "dmsgs/VisionInfo.h"
#include "dvision/line_segment.hpp"
#include "dvision/projection.hpp"

// vision shared info, used in dvision for concurrency

namespace dvision {

struct VisionSharedInfo
{
    dmsgs::VisionInfo visionInfo;
    cv::Mat hsvImg;
    // cv::Mat grayImg;
    cv::Mat guiImg;

    cv::Mat obstacleMask;
    cv::Mat convexHull;
    cv::Mat fieldBinaryRaw;
    std::vector<cv::Point> hullField;
    std::vector<cv::Point2f> fieldHullReal;

    darknet::bbox ballPosition;
    darknet::bbox goalPosition;

    double pitch, yaw;
};
}
