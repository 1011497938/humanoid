/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:24+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: goal_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:23+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "darknetcxx/detector.hpp"
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/kalman.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include <ros/ros.h>
#include <vector>

using dmsgs::VisionInfo;
namespace dvision {
class GoalDetector : public IDetector
{
  public:
    explicit GoalDetector();
    ~GoalDetector();
    bool Init();
    void Detect(const darknet::bbox& goal_bbox,
                const cv::Mat& canny_img,
                const cv::Mat& hsv_img,
                cv::Mat& gui_img,
                const std::vector<cv::Point>& hull_field,
                Projection& projection,
                VisionInfo& vision_info);

    inline std::vector<cv::Point2f> goal_position()
    {
        if (parameters.goal.useKalman) {
            return goal_position_kalman_;
        } else {
            return goal_position_;
        }
    }

  private:
    cv::Rect boundary_rect_;
    ObjectPosKalmanFilter left_goal_kalmanI_;
    ObjectPosKalmanFilter right_goal_kalmanI_;
    std::vector<cv::Point2f> goal_position_;
    std::vector<cv::Point2f> goal_position_kalman_;
    std::vector<cv::Point2f> goal_position_candidate_;

    bool Update();
    bool Process(const darknet::bbox& goal_bbox, const cv::Mat& canny_img, const cv::Mat& hsv_img, cv::Mat& gui_img, const std::vector<cv::Point>& field_hull, Projection& projection);

    void GetGoalPostCandidates(const std::vector<cv::Vec4i>& hough_lines,
                               std::vector<LineSegment>& all_ver_lines,
                               const cv::Mat& canny_img,
                               const cv::Mat& hsv_img,
                               cv::Mat& gui_img,
                               const std::vector<cv::Point>& field_hull,
                               Projection& projection);
    bool VoteGoalPostPoint(LineSegment& tmp_line,
                           const cv::Point2d& point,
                           const double& jump_double,
                           cv::Mat& gui_img,
                           const cv::Mat& raw_hsv,
                           const cv::Mat& canny_img,
                           double& left_avg,
                           double& right_avg,
                           int& vote_for_double_left,
                           int& vote_for_double_right,
                           std::vector<cv::Point>& goal_post_points,
                           const bool& use_canny = true);
    bool GetJumpDouble(const cv::Point& down, double& jump_double, Projection& projection, const std::vector<cv::Point> field_hull);
    bool CheckPointInField(const cv::Point& down, Projection& projection, const std::vector<cv::Point> field_hull);
    void CutOffInvalidDownPoints(LineSegment& tmp_line_changed, std::vector<cv::Point>& goal_post_points);
    void ExtendGoalPosts(const std::vector<LineSegment>& all_ver_lines,
                         std::vector<LineSegment>& all_lines,
                         const cv::Mat& canny_img,
                         const cv::Mat& hsv_img,
                         cv::Mat& gui_img,
                         const std::vector<cv::Point>& field_hull,
                         Projection& projection);
    bool CheckValidGoal(LineSegment& tmp_line, const darknet::bbox& goal_bbox, Projection& projection, const std::vector<cv::Point> field_hull, const float& longest_goal_post_len);
    bool CheckDistanceBox(const cv::Point2f& down_point_in_real, const double& length);
    void CheckGoalWidth();
    void CheckGoalWithDarknet(const darknet::bbox& goal_bbox, Projection& projection, const float& scale);
};
} // namespace dvision
