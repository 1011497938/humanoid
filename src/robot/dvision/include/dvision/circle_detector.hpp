/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:19:44+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: circle_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:02+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include <ros/ros.h>
#include <vector>

using dmsgs::VisionInfo;
namespace dvision {
class CircleDetector : public IDetector
{
  public:
    explicit CircleDetector();
    ~CircleDetector();
    bool Init();
    void Detect(std::vector<LineSegment>& result_lines, Projection& projection, VisionInfo& vision_info);
    bool Process(std::vector<LineSegment>& result_lines, Projection& projection);

    inline cv::Point2f& result_circle()
    {
        return result_circle_;
    }


  private:
    cv::Point2f result_circle_;

};
} // namespace dvision
