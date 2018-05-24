/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-05T19:25:53+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: kalman.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-05T19:25:54+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/parameters.hpp"
#include "dvision/timer.hpp"
#include "opencv2/opencv.hpp"
#include <limits>
#include <vector>

namespace dvision {
#define UNKNOWN_OBJ_POS std::numeric_limits<float>::quiet_NaN()
class ObjectPosKalmanFilter
{
  public:
    explicit ObjectPosKalmanFilter(const cv::Point2f& p);
    ~ObjectPosKalmanFilter();
    void Init(const cv::Point2f& p);
    bool Update(const cv::Point2f& p);

    cv::Point2f GetPrediction();
    cv::Point2f GetResult();
    cv::Point2f GetVelocity();

  private:
    cv::KalmanFilter* kalman_;
    cv::Point2f last_est_;
    cv::Point2f curr_est_;
    cv::Point2f curr_pred_;
    Timer miss_time_;
    Timer diff_time_;
    bool reset_;
    int state_size_;
    int measure_size_;
    int control_size_;
};
} // namespace dvision
