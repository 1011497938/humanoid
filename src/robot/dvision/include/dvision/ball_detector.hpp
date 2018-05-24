/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-10T10:44:45+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: ball_detector.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-10T10:44:58+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "darknetcxx/detector.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include "dvision/utils.hpp"
#include <ros/ros.h>
#include <string>
#include <vector>

using dmsgs::VisionInfo;
namespace dvision {

class BallDetector : public IDetector
{
  public:
    explicit BallDetector();
    ~BallDetector();
    bool Init();
    void Detect(const darknet::bbox& ball_position, cv::Mat& gui_img, std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary, Projection& projection, VisionInfo& vision_info);

    inline cv::Point ball_image()
    {
        return ball_image_;
    }

    inline cv::Point2f ball_field()
    {
        if (parameters.ball.useKalman) {
            return ball_field_kalman_;
        } else {
            return ball_field_;
        }
    }

  private:
    cv::Point ball_image_;
    cv::Point ball_image_top_;
    cv::Point ball_image_bottom_;
    float ball_image_radius_;
    cv::Point2f ball_field_;
    cv::Point2f ball_field_kalman_;
    cv::Point2f ball_velocity_;

    ObjectPosKalmanFilter kalmanI_;
    cv::SimpleBlobDetector::Params blob_params_;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector_;

    bool Update();
    bool Process(const darknet::bbox& ball_position, cv::Mat& gui_img, std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary, Projection& projection);
    bool FindBallCenter(const cv::Mat& field_binary, const cv::Point& ball_image_top, const cv::Point& ball_image_bottom, const float& scale, Projection& projection);
    bool CheckBallInField(std::vector<cv::Point2f>& field_hull_real);
    bool CheckBallDist();
    bool CheckBallRadius(Projection& projection);
};
} // namespace dvision
