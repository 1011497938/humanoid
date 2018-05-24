/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:47:02+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: goal_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:11+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/goal_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
GoalDetector::GoalDetector()
  : left_goal_kalmanI_(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS))
  , right_goal_kalmanI_(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS))
{
}

GoalDetector::~GoalDetector()
{
}

bool
GoalDetector::Init()
{
    // ROS_DEBUG("GoalDetector Init");
    boundary_rect_.x = 0;
    boundary_rect_.y = 0;
    boundary_rect_.width = parameters.camera.width;
    boundary_rect_.height = parameters.camera.height;

    // left_goal_kalmanI_.Init(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
    // right_goal_kalmanI_.Init(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
    return true;
}

void
GoalDetector::Detect(const darknet::bbox& goal_bbox,
                     const cv::Mat& canny_img,
                     const cv::Mat& hsv_img,
                     cv::Mat& gui_img,
                     const std::vector<cv::Point>& hull_field,
                     Projection& projection,
                     VisionInfo& vision_info)
{
    Timer t;
    if (vision_info.see_field) {
        Process(goal_bbox, canny_img, hsv_img, gui_img, hull_field, projection);
        vision_info.see_goal = Update();
    }
    if (vision_info.see_goal) {
        vision_info.goals_field.clear();
        geometry_msgs::Vector3 tmp;
        for (auto& g : goal_position()) {
            tmp.x = g.x;
            tmp.y = g.y;
            vision_info.goals_field.push_back(tmp);
        }
    }
    //    std::cerr << "goal size: " << vision_info.goals_field.size() << std::endl;
    ROS_DEBUG("goal detect used: %lf ms", t.elapsedMsec());
}

bool
GoalDetector::Update()
{
    if (parameters.goal.useKalman) {
        // if see both goal, just update kalman filter for each goal and get correction
        if (goal_position_.size() >= 2) {
            // get left goal kalman correction
            left_goal_kalmanI_.Update(goal_position_[0]);
            goal_position_kalman_.push_back(left_goal_kalmanI_.GetResult());
            // get right goal kalman correction
            right_goal_kalmanI_.Update(goal_position_[1]);
            goal_position_kalman_.push_back(right_goal_kalmanI_.GetResult());
            // ROS_INFO("see both goal (%f, %f) (%f, %f)", goal_position_[0].x, goal_position_[0].y, goal_position_[1].x, goal_position_[1].y);
            // ROS_INFO("see both goal kalman(%f, %f) (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y, goal_position_kalman_[1].x, goal_position_kalman_[1].y);

            // std::cerr << goal_position_[0].x << " " << goal_position_[0].y << " " << goal_position_[1].x << " " << goal_position_[1].y << " " << goal_position_kalman_[0].x << " "
            //           << goal_position_kalman_[0].y << " " << goal_position_kalman_[1].x << " " << goal_position_kalman_[1].y << std::endl;

            return true;
            // } else if (goal_position_.size() == 1) {
            //     // if just see single goal, determine whether it's left goal or right goal
            //     if (GetDistance(goal_position_[0], left_goal_kalmanI_.GetResult()) < parameters.goal.maxUnknownDistError) {
            //         // if unknown goal is near left goal, update left goal kalman filter
            //         // BTW update right goal kalman filter with UNKNOWN_POS
            //
            //         if (right_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS))) {
            //             left_goal_kalmanI_.Update(goal_position_[0]);
            //             goal_position_kalman_.push_back(left_goal_kalmanI_.GetResult());
            //             goal_position_kalman_.push_back(right_goal_kalmanI_.GetResult());
            //         }
            //
            //         // ROS_WARN("see left goal (%f, %f)", goal_position_[0].x, goal_position_kalman_[0].y);
            //         // ROS_WARN("see left goal kalman (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y);
            //         // ROS_WARN("see left goal (%f, %f) (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y, goal_position_kalman_[1].x, goal_position_kalman_[1].y);
            //         return true;
            //     }
            //     if (GetDistance(goal_position_[0], right_goal_kalmanI_.GetResult()) < parameters.goal.maxUnknownDistError) {
            //         // if unknown goal is near right goal, update right goal kalman filter
            //         // BTW update left goal kalman filter with UNKNOWN_POS;
            //
            //         if (left_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS))) {
            //             goal_position_kalman_.push_back(left_goal_kalmanI_.GetResult());
            //             right_goal_kalmanI_.Update(goal_position_[0]);
            //             goal_position_kalman_.push_back(right_goal_kalmanI_.GetResult());
            //         }
            //
            //         // ROS_WARN("see right goal (%f, %f)", goal_position_[0].x, goal_position_[0].y);
            //         // ROS_WARN("see right goal kalman (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y);
            //         // ROS_WARN("see right goal (%f, %f) (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y, goal_position_kalman_[1].x, goal_position_kalman_[1].y);
            //         return true;
            //     }
            //     // otherwise just look it as a unknown goal
            //     // goal_position_kalman_.push_back(goal_position_[0]);
            //     // ROS_ERROR("see unknown goal (%f, %f)", goal_position_kalman_[0].x, goal_position_kalman_[0].y);
            //     left_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
            //     right_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
            //     return false;
        }
        // if goal is not seen, just add reset counter of kalman filter
        // ROS_ERROR("not see goal");
        left_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
        right_goal_kalmanI_.Update(cv::Point2f(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS));
        return false;
    }
    return true;
}

bool
GoalDetector::Process(const darknet::bbox& goal_bbox, const cv::Mat& canny_img, const cv::Mat& hsv_img, cv::Mat& gui_img, const std::vector<cv::Point>& field_hull, Projection& projection)
{
    ROS_DEBUG("goal detector tick");

    if (!parameters.goal.enable) {
        return false;
    }

    // clear
    goal_position_.clear();
    goal_position_kalman_.clear();
    goal_position_candidate_.clear();
    std::vector<LineSegment> result_lines, all_lines;

    // hougg line detection
    std::vector<cv::Vec4i> hough_lines;
    cv::HoughLinesP(canny_img, hough_lines, 1, M_PI / 45, 10, parameters.goal.MinLineLength, parameters.goal.MaxLineGap);

    // get goal post candidates
    std::vector<LineSegment> all_ver_lines;
    GetGoalPostCandidates(hough_lines, all_ver_lines, canny_img, hsv_img, gui_img, field_hull, projection);

    // Extend goal post point
    if (parameters.goal.useGoalPostExtend) {
        ExtendGoalPosts(all_ver_lines, all_lines, canny_img, hsv_img, gui_img, field_hull, projection);
    } else {
        MergeLinesMax(all_ver_lines, parameters.goal.AngleToMerge, parameters.goal.DistanceToMerge, parameters.goal.CollinearLengthToMerge, all_lines, boundary_rect_);
    }

    // get length of longest goal post
    float longest_goal_post_len = 0.0;
    if (!all_lines.empty()) {
        auto longest_goal_post = std::max_element(all_lines.begin(), all_lines.end(), [](const LineSegment& lhs, const LineSegment& rhs) { return lhs.GetLength() < rhs.GetLength(); });
        longest_goal_post_len = longest_goal_post->GetLength();
    }

    // check goal validation
    for (auto& tmp_line : all_lines) {
        // get real coord of down point
        cv::Point down = tmp_line.GetDownPoint();
        cv::Point up = tmp_line.GetUpPoint();
        cv::Point2f down_real;

        if (!CheckValidGoal(tmp_line, goal_bbox, projection, field_hull, longest_goal_post_len)) {
            continue;
        }

        if (!projection.getOnRealCoordinate(down, down_real)) {
            ROS_ERROR("Erorr in programming!");
            return false;
        }

        // add valid down point candidate
        goal_position_candidate_.push_back(down_real);
        // add corresponding goal post
        result_lines.emplace_back(down, up);
    }

    // check width between two goal posts
    if (parameters.goal.useGoalWidthCheck && goal_position_candidate_.size() >= 2) {
        CheckGoalWidth();
    } else {
        goal_position_ = goal_position_candidate_;
    }

    if (parameters.goal.useDarknetCheck) {
        CheckGoalWithDarknet(goal_bbox, projection, parameters.goal.bboxScale);
    }

    // draw all vertical goal lines
    if (parameters.goal.showAllLines && parameters.monitor.update_gui_img) {
        for (auto line : all_ver_lines) {
            cv::line(gui_img, line.P1, line.P2, yellowColor(), 2, 8);
        }
    }

    // draw all possible goal lines
    if (parameters.goal.showAllLines && parameters.monitor.update_gui_img) {
        for (auto line : all_lines) {
            cv::line(gui_img, line.P1, line.P2, darkOrangeColor(), 2, 8);
            // cv::putText(
            //   gui_img, std::to_string(line.GetDownPoint().x) + "," + std::to_string(line.GetDownPoint().y), line.GetDownPoint(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1, CV_AA);
        }
    }

    if (goal_position_.size() > 0) {
        // draw selected goal lines
        if (parameters.goal.showResLine && parameters.monitor.update_gui_img) {
            for (auto line : result_lines) {
                cv::line(gui_img, line.P1, line.P2, redColor(), 2, 8);
            }
        }
    }

    return goal_position_.size() > 0;
}

void
GoalDetector::GetGoalPostCandidates(const std::vector<cv::Vec4i>& hough_lines,
                                    std::vector<LineSegment>& all_ver_lines,
                                    const cv::Mat& canny_img,
                                    const cv::Mat& hsv_img,
                                    cv::Mat& gui_img,
                                    const std::vector<cv::Point>& field_hull,
                                    Projection& projection)
{
    std::vector<cv::Point> goal_post_points;

    for (auto& hough_line : hough_lines) {
        // hough_line (x_1, y_1, x_2, y_2)
        LineSegment tmp_line(cv::Point2d(hough_line[0], hough_line[1]), cv::Point2d(hough_line[2], hough_line[3]));

        // filter vertical lines
        if (tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))) > 15) {
            continue;
        }

        // ignore lines whose up point is in field hull
        cv::Point up = tmp_line.GetUpPoint();
        double up_distance_to_field = cv::pointPolygonTest(field_hull, up, true);
        if (up_distance_to_field >= parameters.goal.MaxOutField + 10) {
            continue;
        }

        // check down points near field hull or far away from it
        cv::Point down = tmp_line.GetDownPoint();
        // double down_distance_to_field = cv::pointPolygonTest(field_hull, down, true);
        // if (down_distance_to_field >= parameters.goal.MaxOutField || down_distance_to_field <= parameters.goal.MinNearFieldUpPoint) {
        //     // pass
        // } else {
        //     continue;
        // }

        // show hough lines
        if (parameters.goal.showHoughLines && parameters.monitor.update_gui_img) {
            cv::line(gui_img, tmp_line.P1, tmp_line.P2, pinkColor(), 1);
        }

        // get jump value
        double jump_double = parameters.goal.jumpMax;
        if (!GetJumpDouble(down, jump_double, projection, field_hull)) {
            continue;
        }

        // vote goal post
        double left_avg = 0;
        double right_avg = 0;
        std::vector<cv::Point2d> midds = tmp_line.GetMidPoints(5);
        int vote_for_double_left = 0;
        int vote_for_double_right = 0;
        goal_post_points.clear();
        for (size_t j = 0; j < midds.size(); j++) {
            if (!VoteGoalPostPoint(tmp_line, midds[j], jump_double, gui_img, hsv_img, canny_img, left_avg, right_avg, vote_for_double_left, vote_for_double_right, goal_post_points)) {
                continue;
            }
        }

        bool left_OK = (vote_for_double_left / static_cast<float>(midds.size())) * 100. > parameters.goal.doubleVote;
        bool right_OK = (vote_for_double_right / static_cast<float>(midds.size())) * 100. > parameters.goal.doubleVote;
        // ROS_INFO("down (%d, %d), vote_left: %d, vote_right: %d", down.x, down.y, vote_for_double_left, vote_for_double_right);
        if (left_OK || right_OK) {
            LineSegment tmp_line_changed = tmp_line;

            // CutOffInvalidDownPoints(tmp_line_changed, goal_post_points);

            // 平移，将球门柱左边缘线或右边缘线向中间平移
            if (left_OK) {
                int amount = std::abs(left_avg / vote_for_double_left) / 2.;
                tmp_line_changed.P1.x -= amount;
                tmp_line_changed.P2.x -= amount;
            } else if (right_OK) {
                int amount = std::abs(right_avg / vote_for_double_right) / 2.;
                tmp_line_changed.P1.x += amount;
                tmp_line_changed.P2.x += amount;
            }
            tmp_line_changed.Clip(boundary_rect_);

            // push tmp_line into all_ver_lines for post-processing
            all_ver_lines.push_back(tmp_line_changed);
        }
    }
}

bool
GoalDetector::VoteGoalPostPoint(LineSegment& tmp_line,
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
                                const bool& use_canny)
{
    // flag for valid goal post point
    bool valid_goal_post_point = false;
    // get perpendicular line segment from specific point with length of jump value
    LineSegment perpendicular_line = tmp_line.PerpendicularLineSegment(jump_double, point);

    // get iterator for left and right perpendicular line segment
    cv::Point left = (perpendicular_line.P1.x < perpendicular_line.P2.x) ? perpendicular_line.P1 : perpendicular_line.P2;
    cv::Point right = (perpendicular_line.P1.x < perpendicular_line.P2.x) ? perpendicular_line.P2 : perpendicular_line.P1;
    cv::LineIterator it_left(canny_img, point, left, 8);
    cv::LineIterator it_right(canny_img, point, right, 8);
    cv::LineIterator it_hsv_left(raw_hsv, point, left, 8);
    cv::LineIterator it_hsv_right(raw_hsv, point, right, 8);

    bool safe_to_show = perpendicular_line.P1.x >= 0 && perpendicular_line.P1.y >= 0 && perpendicular_line.P1.x < parameters.camera.width && perpendicular_line.P1.y < parameters.camera.height &&
                        perpendicular_line.P2.x >= 0 && perpendicular_line.P2.y >= 0 && perpendicular_line.P2.x < parameters.camera.width && perpendicular_line.P2.y < parameters.camera.height;

    for (int k = 0; k < it_left.count; k++, ++it_left, ++it_hsv_left) {
        if (k < 2)
            continue;
        uchar canny_res = *(*it_left);
        cv::Vec3b hsvC = (cv::Vec3b)*it_hsv_left;

        // if getting high response in canny image && distance > minDoubleLength
        // Canny高响应（如球门柱左边缘线上的某个等分点的垂线段碰到了右边缘线）
        // 且左右边缘线像素距离大于minDoubleLength
        if ((canny_res > 0 || !use_canny) && k > parameters.goal.minDoubleLength) {
            if (safe_to_show && parameters.monitor.update_gui_img && parameters.goal.showVote) {
                cv::line(gui_img, point, it_hsv_left.pos(), darkOrangeColor(), 1);
            }
            // 用以之后计算平移距离
            left_avg += k;
            // 投票也有我一份
            vote_for_double_left++;
            valid_goal_post_point = true;
            break;
        }

        // 像素点的HSV不在预设范围内则gg
        if (hsvC[0] >= parameters.goal.h0 && hsvC[0] <= parameters.goal.h1 && hsvC[1] >= parameters.goal.s0 && hsvC[1] <= parameters.goal.s1 && hsvC[2] >= parameters.goal.v0 &&
            hsvC[2] <= parameters.goal.v1) {
        } else {
            break;
        }
    }

    // the same
    for (int k = 0; k < it_right.count; k++, ++it_right, ++it_hsv_right) {
        if (k < 2)
            continue;
        uchar canny_res = *(*it_right);
        cv::Vec3b hsvC = (cv::Vec3b)*it_hsv_right;

        if ((canny_res > 0 || !use_canny) && k > parameters.goal.minDoubleLength) {
            if (safe_to_show && parameters.monitor.update_gui_img && parameters.goal.showVote) {
                cv::line(gui_img, point, it_hsv_right.pos(), darkOrangeColor(), 1);
            }
            right_avg += k;
            vote_for_double_right++;
            valid_goal_post_point = true;
            break;
        }

        if (hsvC[0] >= parameters.goal.h0 && hsvC[0] <= parameters.goal.h1 && hsvC[1] >= parameters.goal.s0 && hsvC[1] <= parameters.goal.s1 && hsvC[2] >= parameters.goal.v0 &&
            hsvC[2] <= parameters.goal.v1) {
        } else {
            break;
        }
    }

    // 如果left或者right满足条件，则将此点加入goal_post_points中待用
    if (valid_goal_post_point) {
        goal_post_points.push_back(point);
    }

    return true;
}

bool
GoalDetector::GetJumpDouble(const cv::Point& down, double& jump_double, Projection& projection, const std::vector<cv::Point> field_hull)
{
    // get real coord of down point
    cv::Point2f down_real;
    if (!projection.getOnRealCoordinate(down, down_real)) {
        ROS_ERROR("Erorr in programming!");
        return false;
    }

    // get jump value varied from distance between goal post and robot
    double distance = GetDistance(down_real);
    if (distance < parameters.goal.jumpDistanceNear) {
        jump_double = parameters.goal.jumpDoubleNear;
    } else if (distance >= parameters.goal.jumpDistanceNear && distance < parameters.goal.jumpDistanceFar) {
        jump_double = parameters.goal.jumpDoubleMid;
    } else {
        jump_double = parameters.goal.jumpDoubleFar;
    }
    return true;
}

bool
GoalDetector::CheckPointInField(const cv::Point& down, Projection& projection, const std::vector<cv::Point> field_hull)
{
    // down point must be above the center of field hull
    // if (down.y > field_hull_center.y / 2) {
    //     return false;
    // }

    // get real coord of down point
    cv::Point2f down_real;
    if (!projection.getOnRealCoordinate(down, down_real)) {
        ROS_ERROR("Erorr in programming!");
        return false;
    }

    // get offset varied from distance between goal post and robot
    float distance = GetDistance(down_real);
    float MaxOutField, MinOutField;
    if (distance < parameters.goal.OutFieldDistanceNear) {
        MaxOutField = parameters.goal.MaxOutField + parameters.goal.OutFieldOffsetNear;
        MinOutField = parameters.goal.MinOutField + parameters.goal.OutFieldOffsetNear;
    } else if (distance >= parameters.goal.OutFieldDistanceNear && distance < parameters.goal.OutFieldDistanceFar) {
        MaxOutField = parameters.goal.MaxOutField + parameters.goal.OutFieldOffsetMid;
        MinOutField = parameters.goal.MinOutField + parameters.goal.OutFieldOffsetMid;
    } else {
        MaxOutField = parameters.goal.MaxOutField + parameters.goal.OutFieldOffsetFar;
        MinOutField = parameters.goal.MinOutField;
    }

    // check if down point is near field hull
    double down_distance_to_field = cv::pointPolygonTest(field_hull, down, true);
    if (down_distance_to_field >= MaxOutField && down_distance_to_field <= MinOutField) {
        return true;
    } else {
        // ROS_DEBUG("down_distance_to_field %f, distance to robot %f, MaxOutField %f, MinOutField %f", down_distance_to_field, distance, MaxOutField, MinOutField);
        return false;
    }
}

void
GoalDetector::CutOffInvalidDownPoints(LineSegment& tmp_line_changed, std::vector<cv::Point>& goal_post_points)
{
    // 球门柱下端点下探避免
    cv::Point valid_down = tmp_line_changed.GetDownPoint();
    // 按y升序排列
    std::sort(goal_post_points.begin(), goal_post_points.end(), [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
    // 如果line的下端点与高响应点距离大于10
    // 则将line的下端点换为高响应点集合的下端点
    if (GetDistance(goal_post_points.back(), valid_down) > parameters.goal.cutOffInvalidPoints) {
        valid_down = goal_post_points.back();
    }
    // 如果一个高响应点与其下的高响应点的距离大于10
    // 即出现goal post下探到禁区线的情况
    // 则将line的下端点换为该高响应点
    for (size_t l = goal_post_points.size() - 1; l > 0; --l) {
        if (GetDistance(goal_post_points[l], goal_post_points[l - 1]) > parameters.goal.cutOffInvalidPoints) {
            valid_down = goal_post_points[l];
        }
    }
    // 替换line的下端点
    tmp_line_changed.SetDownPoint(valid_down);
}

void
GoalDetector::ExtendGoalPosts(const std::vector<LineSegment>& all_ver_lines,
                              std::vector<LineSegment>& all_lines,
                              const cv::Mat& canny_img,
                              const cv::Mat& hsv_img,
                              cv::Mat& gui_img,
                              const std::vector<cv::Point>& field_hull,
                              Projection& projection)
{
    // merge lines
    std::vector<LineSegment> all_ver_lines_2, all_ver_lines_3;
    MergeLinesMax(all_ver_lines, parameters.goal.AngleToMerge, parameters.goal.DistanceToMerge, parameters.goal.CollinearLengthToMerge, all_ver_lines_2, boundary_rect_);

    // extend
    for (auto& tmp_line : all_ver_lines_2) {
        cv::Point up = tmp_line.GetUpPoint();
        cv::Point down = tmp_line.GetDownPoint();

        // chekc if it's vertical
        if (tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))) > 15) {
            // ROS_DEBUG("Angle too large (%f)", tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))));
            continue;
        }

        // check whether to extend up or down
        bool extendUp = false;
        bool extendDown = false;
        double down_distance_to_field = cv::pointPolygonTest(field_hull, down, true);
        if (down_distance_to_field <= parameters.goal.MinNearFieldUpPoint) {
            // if down point is enough far away from field hull
            // then extend its down point
            extendDown = true;
            // cv::circle(gui_img, down, 3, cv::Scalar(0, 0, 0), 2);
            // ROS_DEBUG("Goal post extend down from (%d, %d)", down.x, down.y);
            // } else if (CheckPointInField(down, projection, field_hull)) {
        } else if (down_distance_to_field >= parameters.goal.MaxOutField) {
            // if down point is near the border of field hull
            // then extend its up point
            extendUp = true;
            // cv::circle(gui_img, up, 3, cv::Scalar(0, 0, 0), 2);
            // ROS_DEBUG("Goal post extend up from (%d, %d)", up.x, up.y);
        } else {
            // ROS_DEBUG("Goal post no extending");
            continue;
        }

        // extend goal post when possible
        std::vector<cv::Point> goal_post_points;
        int cnt_invalid_points = 0;
        int cnt_total_ext_points = 0;
        cv::Point2f extension_point;
        // LineSegment tmp_line_ext(down, up);
        LineSegment tmp_line_ext(down, up);

        while (cnt_invalid_points <= parameters.goal.extInvalidPoints && cnt_total_ext_points <= parameters.goal.extTotalPoints) {
            if (extendDown) {
                // extension_point = cv::Point(tmp_line_ext.GetDownPoint().x, tmp_line_ext.GetDownPoint().y + parameters.goal.extLengthPerAttempt);
                extension_point = tmp_line.ExtensionPointDown(parameters.goal.extLengthPerAttempt);
                // std::cout << "Original Down (" << tmp_line_ext.GetDownPoint().x << "," << tmp_line_ext.GetDownPoint().y << ")" << std::endl;
                // std::cout << "Try Down Expand to (" << extension_point.x << "," << extension_point.y << ")" << std::endl;
                // tmp_line_ext.SetDownPoint(extension_point);
                tmp_line_ext = tmp_line.ExtensionCordDown(parameters.goal.extLengthPerAttempt);
            } else if (extendUp) {
                // extension_point = cv::Point(tmp_line_ext.GetUpPoint().x, tmp_line_ext.GetUpPoint().y - parameters.goal.extLengthPerAttempt);
                extension_point = tmp_line.ExtensionPointUp(parameters.goal.extLengthPerAttempt);
                // std::cout << "Original Up (" << tmp_line_ext.GetUpPoint().x << "," << tmp_line_ext.GetUpPoint().y << ")" << std::endl;
                // std::cout << "Try Up Expand to (" << extension_point.x << "," << extension_point.y << ")" << std::endl;
                // tmp_line_ext.SetUpPoint(extension_point);
                tmp_line_ext = tmp_line.ExtensionCordUp(parameters.goal.extLengthPerAttempt);
            }

            // break when extend to boundary
            if (extension_point.y <= 0 || extension_point.y >= parameters.camera.height - 1) {
                break;
            }

            // get jump value
            double jump_double = parameters.goal.jumpMax;
            if (!GetJumpDouble(extension_point, jump_double, projection, field_hull)) {
                break;
            }

            // check if the extending point is valid
            double left_avg = 0;
            double right_avg = 0;
            int vote_for_double_left = 0;
            int vote_for_double_right = 0;
            int last_goal_post_points_size = goal_post_points.size();
            std::vector<cv::Point2d> midds = tmp_line_ext.GetMidPoints(5);

            for (size_t j = 0; j < midds.size(); j++) {
                if (!VoteGoalPostPoint(tmp_line, midds[j], jump_double, gui_img, hsv_img, canny_img, left_avg, right_avg, vote_for_double_left, vote_for_double_right, goal_post_points)) {
                    continue;
                }
            }

            // extend line if point is valid
            bool left_OK = (vote_for_double_left / static_cast<float>(midds.size())) * 100. > parameters.goal.doubleVote;
            bool right_OK = (vote_for_double_right / static_cast<float>(midds.size())) * 100. > parameters.goal.doubleVote;
            if (left_OK || right_OK) {
                //     // if (extendDown && GetDistance(tmp_line.GetDownPoint(), extension_point) < parameters.goal.extDownMaxGap) {
                if (extendDown) {
                    if (left_OK) {
                        int amount = std::abs(left_avg / vote_for_double_left) / 2.;
                        extension_point.x -= amount;
                    } else if (right_OK) {
                        int amount = std::abs(right_avg / vote_for_double_right) / 2.;
                        extension_point.x += amount;
                    }
                    tmp_line.SetDownPoint(extension_point);
                    tmp_line.Clip(boundary_rect_);
                    // std::cout << "Down Expand to (" << extension_point.x << "," << extension_point.y << ")" << std::endl;
                    if (parameters.goal.showExtendPoints && parameters.monitor.update_gui_img) {
                        cv::circle(gui_img, extension_point, 3, blueMeloColor(), 2);
                    }
                    cnt_invalid_points += parameters.goal.extLengthPerAttempt - (goal_post_points.size() - last_goal_post_points_size);
                } else if (extendUp) {
                    if (left_OK) {
                        int amount = std::abs(left_avg / vote_for_double_left) / 2.;
                        extension_point.x -= amount;
                    } else if (right_OK) {
                        int amount = std::abs(right_avg / vote_for_double_right) / 2.;
                        extension_point.x += amount;
                    }
                    tmp_line.SetUpPoint(extension_point);
                    tmp_line.Clip(boundary_rect_);
                    // std::cout << "Up Expand to (" << extension_point.x << "," << extension_point.y << ")" << std::endl;
                    if (parameters.goal.showExtendPoints && parameters.monitor.update_gui_img) {
                        cv::circle(gui_img, extension_point, 3, blueColor(), 2);
                    }
                    cnt_invalid_points += parameters.goal.extLengthPerAttempt - (goal_post_points.size() - last_goal_post_points_size);
                }
            } else if (goal_post_points.size() - last_goal_post_points_size > 0) {
                if (extendDown) {
                    tmp_line.SetDownPoint(goal_post_points.back());
                    if (parameters.goal.showExtendPoints && parameters.monitor.update_gui_img) {
                        cv::circle(gui_img, goal_post_points.back(), 3, blueMeloColor(), 2);
                    }
                    cnt_invalid_points += parameters.goal.extLengthPerAttempt - (goal_post_points.size() - last_goal_post_points_size);
                } else if (extendUp) {
                    tmp_line.SetUpPoint(goal_post_points.back());
                    // std::cout << "Up Expand to (" << extension_point.x << "," << extension_point.y << ")" << std::endl;
                    if (parameters.goal.showExtendPoints && parameters.monitor.update_gui_img) {
                        cv::circle(gui_img, goal_post_points.back(), 3, blueColor(), 2);
                    }
                    cnt_invalid_points += parameters.goal.extLengthPerAttempt - (goal_post_points.size() - last_goal_post_points_size);
                }
                cnt_invalid_points += parameters.goal.extLengthPerAttempt;
            }
            cnt_total_ext_points += parameters.goal.extLengthPerAttempt;
        }

        // std::cout << "goal post points size: " << goal_post_points.size() << std::endl;
        if (goal_post_points.size() > 0) {
            CutOffInvalidDownPoints(tmp_line, goal_post_points);
        }

        // check length of the extended line
        if (tmp_line.GetLength() >= parameters.goal.extValidLength) {

            all_ver_lines_3.push_back(tmp_line);
        }
    }

    // merge extended lines
    MergeLinesMax(all_ver_lines_3, parameters.goal.AngleToMerge, parameters.goal.DistanceToMerge, parameters.goal.CollinearLengthToMerge, all_lines, boundary_rect_);
}

bool
GoalDetector::CheckValidGoal(LineSegment& tmp_line, const darknet::bbox& goal_bbox, Projection& projection, const std::vector<cv::Point> field_hull, const float& longest_goal_post_len)
{
    cv::Point up = tmp_line.GetUpPoint();
    cv::Point down = tmp_line.GetDownPoint();

    // check if it's vertical
    if (tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))) > 15) {
        // ROS_WARN("Angle (%f) too large (%d, %d)", tmp_line.GetAbsMinAngleDegree(LineSegment(cv::Point(0, 0), cv::Point(0, 100))), down.x, down.y);
        return false;
    }

    // check goal post length
    if (parameters.goal.useGoalLengthCheck && tmp_line.GetLength() < parameters.goal.validGoalLengthCoff * longest_goal_post_len) {
        // ROS_WARN("Goal: not enough long (%f << %f) down(%d, %d)", tmp_line.GetLength(), parameters.goal.validGoalLengthCoff * longest_goal_post_len, down.x, down.y);
        return false;
    }

    // check if up point of goal post is far away enough from field hull
    float up_distance_to_field = cv::pointPolygonTest(field_hull, up, true);
    if (up_distance_to_field > parameters.goal.MinNearFieldUpPoint) {
        // ROS_WARN("Goal: up (%d, %d) in field (distance: %f)", up.x, up.y, up_distance_to_field);
        return false;
    }

    // get real coord of down point (too frequent...)
    cv::Point2f down_real;
    if (!projection.getOnRealCoordinate(down, down_real)) {
        ROS_ERROR("Erorr in programming!");
        return false;
    }

    // check if down point of goal post is near field hull
    if (!CheckPointInField(down, projection, field_hull)) {
        // float down_distance_to_field = cv::pointPolygonTest(field_hull, down, true);
        // ROS_WARN("Goal: down (%d, %d) not in field (distance from field: %f ,real_down: %f)", down.x, down.y, down_distance_to_field, GetDistance(down_real));
        return false;
        // } else {
        //     float down_distance_to_field = cv::pointPolygonTest(field_hull, down, true);
        //     ROS_WARN("Goal: down (%d, %d) in field (distance: %f, down real (%f, %f))", down.x, down.y, down_distance_to_field, down_real.x, down_real.y);
    }

    // check goal post is in the field of robot vision
    // double ver_len = tmp_line.GetLength();
    // if (!CheckDistanceBox(down_real, ver_len)) {
    //     ROS_WARN("Goal: check box error");
    //     return false;
    // }

    // invalid if goal post is too far away from robot
    if (GetDistance(down_real) > parameters.goal.maxDistFromRobot) {
        // ROS_WARN("Goal down(%d, %d): too far from robot (%f)", down.x, down.y, GetDistance(down_real));
        return false;
    }

    return true;
}

bool
GoalDetector::CheckDistanceBox(const cv::Point2f& down_point_in_real, const double& length)
{
    LineSegment lower_bound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMinLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMinLen));
    LineSegment higher_bound(cv::Point2f(parameters.goal.NearestDistance, parameters.goal.NearMaxLen), cv::Point2f(parameters.goal.FurthestDistance, parameters.goal.FarMaxLen));
    LinearBoundaryChecker checker(lower_bound, higher_bound);

    double distance_to_robot = GetDistance(down_point_in_real);

    return checker.CheckInside(distance_to_robot, length);
}

void
GoalDetector::CheckGoalWidth()
{
    find_pairwise(goal_position_candidate_.begin(), goal_position_candidate_.end(), [&](cv::Point2f& lhs, cv::Point2f& rhs) {
        float goal_dis = GetDistance(lhs, rhs);
        if (goal_dis >= parameters.field_model.goal_width * parameters.goal.minGoalWidthRatio && goal_dis <= parameters.field_model.goal_width * parameters.goal.maxGoalWidthRatio) {
            // ROS_INFO("valid goal dis: %f", goal_dis);
            if (lhs.y >= rhs.y) {
                goal_position_.push_back(lhs);
                goal_position_.push_back(rhs);
            } else {
                goal_position_.push_back(rhs);
                goal_position_.push_back(lhs);
            }
            // ROS_ERROR("get both goal (%f, %f) (%f, %f)", goal_position_[0].x, goal_position_[0].y, goal_position_[1].x, goal_position_[1].y);
        } else {
            // ROS_WARN("illegal goal dis: %f", goal_dis);
        }
    });
}

void
GoalDetector::CheckGoalWithDarknet(const darknet::bbox& goal_bbox, Projection& projection, const float& scale)
{
    // enlarge bbox with scale
    float box_width = (goal_bbox.m_right - goal_bbox.m_left) * scale;
    float box_height = (goal_bbox.m_bottom - goal_bbox.m_top) * scale;
    cv::Point bbox_top(goal_bbox.m_left - box_width * (scale - 1.0) / 2.0, goal_bbox.m_top - box_height * (scale - 1.0) / 2.0);
    cv::Point bbox_bottom(goal_bbox.m_left + box_width * scale, goal_bbox.m_top + box_height * scale);
    boundry_n(bbox_top.x, 0, parameters.camera.width - 1);
    boundry_n(bbox_top.y, 0, parameters.camera.height - 1);
    boundry_n(bbox_bottom.x, 0, parameters.camera.width - 1);
    boundry_n(bbox_bottom.y, 0, parameters.camera.height - 1);

    cv::Rect goal_bbox_rect(bbox_top, bbox_bottom);

    // remove invalid goal post if it's out of goal_bbox_rect
    goal_position_.erase(std::remove_if(goal_position_.begin(),
                                        goal_position_.end(),
                                        [&](const cv::Point2f& goal_real) {
                                            cv::Point goal_image;
                                            projection.getOnImageCoordinate(goal_real, goal_image);
                                            return GetDistance(goal_real) >= parameters.goal.minDarknetResultDist && goal_bbox_rect.contains(goal_image);
                                        }),
                         goal_position_.end());
}

} // namespace dvision
