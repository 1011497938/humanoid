/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-07-14T19:54:10+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: obstacle_detector.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-07-14T20:11:18+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include <string>
#include <vector>

namespace dvision {
using dmsgs::VisionInfo;
class ObstacleObject
{
  public:
    explicit ObstacleObject(cv::Point2f position, float confidence = 1.0, int id = 0);
    ~ObstacleObject();

    inline cv::Point2f position()
    {
        return position_;
    }

    inline float confidence()
    {
        boundry_n(confidence_, 0.0, 1.0);
        return confidence_;
    }

    inline int id()
    {
        return id_;
    }

    bool DecayConfidence();
    bool Update(cv::Point2f pos, float conf);

  private:
    cv::Point2f position_;
    float confidence_;
    int id_;
};

class ObstacleDetector : public IDetector
{
  public:
    explicit ObstacleDetector();
    ~ObstacleDetector();

    bool Init();
    void Detect(const cv::Mat& hsv_img, cv::Mat& obstacle_binary, cv::Mat& gui_img, const cv::Mat& field_convex_hull, Projection& projection, VisionInfo& vision_info);

    inline std::vector<cv::Point2f> obstacle_points()
    {
        return obstacle_points_;
    }

    inline std::vector<ObstacleObject> obstacle_objects()
    {
        return obstacle_objects_;
    }

    inline cv::Mat obstacle_mask()
    {
        return obstacle_mask_;
    }

  private:
    std::vector<cv::Point2f> obstacle_points_;
    std::vector<ObstacleObject> obstacle_objects_;
    cv::Mat obstacle_mask_;

    void Update();
    bool Process(cv::Mat& obstacle_binary, cv::Mat& gui_img, Projection& projection);
    void GetBinary(const cv::Mat& hsv_img, const cv::Mat& field_convex_hull, cv::Mat& obstacle_binary, const bool& in_template = false);
};

} // namespace dvision
