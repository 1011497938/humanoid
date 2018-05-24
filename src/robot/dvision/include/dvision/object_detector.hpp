/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-28T12:57:06+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: object_detecor.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-28T12:57:33+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "darknetcxx/detector.hpp"
#include "dvision/idetector.hpp"
#include "dvision/timer.hpp"
#include <string>
#include <vector>

namespace dvision {
class ObjectDetector : public IDetector
{
  public:
    explicit ObjectDetector();
    ~ObjectDetector();
    bool Init();
    void Detect(const cv::Mat& frame, cv::Mat& gui_img);
    bool Process(const cv::Mat& frame, cv::Mat& gui_img);

    inline std::vector<darknet::bbox>& object_position()
    {
        return object_position_;
    }

    inline darknet::bbox& ball_position()
    {
        if (ball_detected_) {
            return ball_position_;
        } else {
            return unknown_position_;
        }
    }

    inline darknet::bbox& goal_position()
    {
        if (goal_detected_) {
            return goal_position_;
        } else {
            return unknown_position_;
        }
    }

    inline std::vector<darknet::bbox>& obstacle_positions()
    {
        return obstacle_positions_;
    }

  private:
    darknet::Network* net_;
    std::vector<std::string> label_list_;
    darknet::Image raw_img_;

    std::vector<darknet::bbox> object_position_;
    std::vector<darknet::RelativeBBox> object_position_relative_;

    darknet::bbox ball_position_, goal_position_;
    darknet::bbox unknown_position_;
    std::vector<darknet::bbox> obstacle_positions_;

    bool ball_detected_, goal_detected_, obstacle_detected_;

    bool CvtRelativePosition();
    bool ReInit();
    bool CheckValidScale(const int& left, const int& right, const int& top, const int& bottom, const float& scale_coff);
    bool CheckValidRatio(const int& left, const int& right, const int& top, const int& bottom, const float& low_ratio, const float& high_ratio);
};
} // namespace dvision
