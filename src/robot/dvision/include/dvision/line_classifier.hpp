#pragma once
#include "dmsgs/VisionInfo.h"
#include "dvision/kalman.hpp"
#include "dvision/line_segment.hpp"
#include "dvision/parameters.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include "dvision/utils.hpp"
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace dvision {
using dmsgs::VisionInfo;

class LineClassifier
{
  public:
    explicit LineClassifier();
    ~LineClassifier();


    bool Init();
    bool Update();
    bool Process(std::vector<LineSegment>& good_lines,
                 const cv::Point2f& result_circle,
                 const std::vector<cv::Point2f>& goal_position,
                 Projection& projection,
                 VisionInfo& vision_info,
                 const double& vision_yaw);

  private:
    std::vector<LineSegment> center_lines;
    std::vector<LineSegment> goal_lines;
    std::vector<LineSegment> other_lines;
    std::vector<double> yaw_correct_bias;

    double CalYawBias();


};
} // namespace dvision
