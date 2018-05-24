
#include "dvision/line_classifier.hpp"
#include "dvision/parameters.hpp"

namespace dvision {

LineClassifier::LineClassifier()
{
}

LineClassifier::~LineClassifier()
{
}

bool
LineClassifier::Init()
{
    // loc_cnt = 0;
    ROS_INFO("LineClassifier Init() finished");
    return true;
}

bool
LineClassifier::Update()
{
    return true;
}

bool
LineClassifier::Process(std::vector<LineSegment>& good_lines,
                        const cv::Point2f& result_circle,
                        const std::vector<cv::Point2f>& goal_position,
                        Projection& projection,
                        VisionInfo& vision_info,
                        const double& vision_yaw)
{
    if (!parameters.line_classifier.enable) {
        return false;
    }
    Timer t;
    center_lines.clear();
    goal_lines.clear();
    other_lines.clear();

    // cv::Mat m_linec_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    if (vision_info.see_field && vision_info.see_line) {
        std::vector<LineSegment> good_lines_rotated = projection.RotateTowardHeading(good_lines);

        // drawContours
        // double offssx = 100;
        // double offssy = 250;
        // double ratioo = 0.3;
        // std::vector<cv::Point> field_hull_real_rotated_ratio;
        // for (size_t i = 0; i < field_hull_real_rotated.size(); i++) {
        //     field_hull_real_rotated_ratio.push_back(cv::Point(field_hull_real_rotated[i].x * ratioo + offssx, -field_hull_real_rotated[i].y * ratioo + offssy));
        // }
        // std::vector<std::vector<cv::Point>> hulls_real(1, field_hull_real_rotated_ratio);
        // cv::drawContours(m_linec_img, hulls_real, -1, orangeColor(), 1, 8);

        LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
        LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

        for (size_t i = 0; i < good_lines_rotated.size(); i++) {
            LineSegment line_seg = good_lines_rotated[i];
            // res_lines.push_back(line_seg);
            if (line_seg.GetLength() > parameters.line_classifier.minLineLen) {
                // 取中点mid
                // cv::Point2d mid = line_seg.GetMiddle();
                // 线片段与VerLine竖直线夹角小于45度
                if (line_seg.GetAbsMinAngleDegree(VerLine) < 30) {
                    other_lines.push_back(line_seg);
                    // 添加竖直线
                } else if (line_seg.GetAbsMinAngleDegree(HorLine) < 30) {
                    // 添加水平线
                    // 检测到中心圆，且线片段到其距离小于30cm
                    if (vision_info.see_circle && DistanceFromLineSegment(line_seg, projection.RotateTowardHeading(result_circle)) < 30) {
                        center_lines.push_back(line_seg);

                        // // 增加视觉 对 陀螺仪的修正
                        double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
                        if (angle_diff_hor < -90)
                            angle_diff_hor += 180;
                        if (angle_diff_hor > 90)
                            angle_diff_hor += -180;
                        yaw_correct_bias.push_back(angle_diff_hor);

                        center_lines.push_back(line_seg);

                    } else if (goal_position.size() > 0 && line_seg.DistanceFromLine(cv::Point2f(0.0, 0.0)) <= 300 && line_seg.GetLength() >= 120) {
                        bool is_goal_line = false;
                        // 旋转球门柱
                        std::vector<cv::Point2f> goal_position_rotated;
                        goal_position_rotated = projection.RotateTowardHeading(goal_position);
                        // 其实可以写成for循环的
                        if (goal_position_rotated.size() == 2) {
                            is_goal_line = line_seg.DistanceFromLine(goal_position_rotated[0]) < parameters.line_classifier.maxDistBothGoal &&
                                           line_seg.DistanceFromLine(goal_position_rotated[1]) < parameters.line_classifier.maxDistBothGoal;

                        } else if (goal_position_rotated.size() == 1) {
                            is_goal_line = line_seg.DistanceFromLine(goal_position_rotated[0]) < parameters.line_classifier.maxDistSingleGoal;
                        }
                        if (is_goal_line) {
                            double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
                            if (angle_diff_hor < -90)
                                angle_diff_hor += 180;
                            if (angle_diff_hor > 90)
                                angle_diff_hor += -180;
                            yaw_correct_bias.push_back(angle_diff_hor);
                            goal_lines.push_back(line_seg);
                        } else {
                            other_lines.push_back(line_seg);
                        }

                    } else {
                        other_lines.push_back(line_seg);
                    }
                    if (yaw_correct_bias.size() >= parameters.line_classifier.yawCorrectNum) {
                        projection.SetHeadingOffsetBias(CalYawBias());
                    }
                }
            }
            // 线片段长度大于预设值
        }

        // cv::line(m_linec_img, cv::Point(offssx, offssy), cv::Point(50 + offssx, offssy), yellowColor(), 2, 8);
        // cv::line(m_linec_img, cv::Point(offssx, offssy), cv::Point(offssx, -50 + offssy), redColor(), 2, 8);
        // for (size_t i = 0; i < other_lines.size(); ++i) {
        //
        //     cv::line(m_linec_img,
        //              cv::Point(other_lines[i].P1.x * ratioo + offssx, -other_lines[i].P1.y * ratioo + offssy),
        //              cv::Point(other_lines[i].P2.x * ratioo + offssx, -other_lines[i].P2.y * ratioo + offssy),
        //              cv::Scalar(150, 145, 151),
        //              2,
        //              8);
        // }
        // for (size_t i = 0; i < center_lines.size(); ++i) {
        //
        //     cv::line(m_linec_img,
        //              cv::Point(center_lines[i].P1.x * ratioo + offssx, -center_lines[i].P1.y * ratioo + offssy),
        //              cv::Point(center_lines[i].P2.x * ratioo + offssx, -center_lines[i].P2.y * ratioo + offssy),
        //              cv::Scalar(0, 255, 255),
        //              2,
        //              8);
        // }
        //
        // for (size_t i = 0; i < goal_lines.size(); ++i) {
        //     cv::line(m_linec_img,
        //              cv::Point(goal_lines[i].P1.x * ratioo + offssx, -goal_lines[i].P1.y * ratioo + offssy),
        //              cv::Point(goal_lines[i].P2.x * ratioo + offssx, -goal_lines[i].P2.y * ratioo + offssy),
        //              cv::Scalar(0, 0, 255),
        //              2,
        //              8);
        // }
        //
        // cv::imshow("m_linec_img", m_linec_img);
    }
    ROS_DEBUG("line classifier used: %lf ms", t.elapsedMsec());
    return true;
}

double
LineClassifier::CalYawBias()
{
    double bias_sum = 0; // in degree
    double bias_avg = 0;
    int bias_cnt = 0;
    for (size_t i = 0; i < yaw_correct_bias.size(); i++) {
        bias_sum += yaw_correct_bias[i];
    }
    bias_avg = bias_sum / yaw_correct_bias.size();
    bias_sum = 0;
    for (size_t i = 0; i < yaw_correct_bias.size(); i++) {
        if (std::abs(yaw_correct_bias[i] - bias_avg) <= 10) {
            bias_sum += yaw_correct_bias[i];
            bias_cnt++;
        }
    }
    if (bias_cnt > 0) {
        bias_avg = bias_sum / bias_cnt;
    } else {
        bias_avg = 0;
    }
//    std::cout << "bias_avg------------------------------------------------: " << bias_avg << std::endl;
    yaw_correct_bias.clear();
    return bias_avg;
}

} // namespace dvision
