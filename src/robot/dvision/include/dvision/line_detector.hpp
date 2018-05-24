/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:33+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: line_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:25+08:00
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
class LineDetector : public IDetector
{
  public:
    explicit LineDetector();
    ~LineDetector();
    bool Init();
    void Detect(cv::Mat& canny_img,
                cv::Mat& hsv_img,
                // cv::Mat& gray_img,
                cv::Mat& gui_img,
                cv::Mat& field_convex_hull,
                cv::Mat& field_binary_raw,
                std::vector<cv::Point2f>& field_hull_real,
                cv::Mat& m_obstacle_binary,
                Projection& projection,
                VisionInfo& vision_info,
                double in_pitch);
    // bool Process(cv::Mat& canny_img, cv::Mat& hsv_img, cv::Mat& gray_img, cv::Mat& gui_img, cv::Mat& field_convex_hull, cv::Mat& field_binary_raw, std::vector<cv::Point2f>& field_hull_real, cv::Mat& m_obstacle_binary, Projection& projection, double in_pitch);
    bool Process(cv::Mat& canny_img, cv::Mat& hsv_img, cv::Mat& gui_img, cv::Mat& field_convex_hull, cv::Mat& field_binary_raw, std::vector<cv::Point2f>& field_hull_real, cv::Mat& m_obstacle_binary, Projection& projection, double in_pitch);

    // bool
    bool GetLines(cv::Mat& raw_hsv, cv::Mat& field_mask, cv::Mat& gui_img, const bool& SHOWGUI, const cv::Mat& line_binary, std::vector<LineSegment>& res_lines);

    inline std::vector<LineSegment>& result_lines()
    {
        return result_lines_;
    }
    inline std::vector<LineSegment>& good_lines()
    {
        return good_lines_;
    }

  private:
    std::vector<LineSegment> bad_lines_;
    std::vector<LineSegment> good_lines_;
    std::vector<LineSegment> result_lines_;

};
} // namespace dvision
