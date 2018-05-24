/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T18:49:17+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: circle_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:20+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/circle_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
CircleDetector::CircleDetector()
{
}

CircleDetector::~CircleDetector()
{
}

bool
CircleDetector::Init()
{
    ROS_DEBUG("CircleDetector Init");
    return true;
}

void
CircleDetector::Detect(std::vector<LineSegment>& result_lines, Projection& projection, VisionInfo& vision_info)
{

    Timer t;
    if (vision_info.see_field && vision_info.see_line) {
        vision_info.see_circle = Process(result_lines, projection);
    }
    if (vision_info.see_circle) {
        // cv::Point2d circle_global = getOnGlobalCoordinate(m_loc.location(), m_circle.result_circle());
        // cv::Point2d circle_global = getOnGlobalCoordinate(m_loc.location(), m_circle.rotated_circle());
        vision_info.circle_field.x = result_circle().x;
        vision_info.circle_field.y = result_circle().y;
    }
    ROS_DEBUG("circle detect used: %lf ms", t.elapsedMsec());
}

bool
CircleDetector::Process(std::vector<LineSegment>& result_lines, Projection& projection)
{
    // ROS_DEBUG("CircleDetector Tick");
    // check if enabled
    if (!parameters.circle.enable) {
        return false;
    }

    std::vector<cv::Point2f> circle_point_candidates;
    // traverse all lines in result_lines and get candidates of circle points
    for (size_t i = 0; i < result_lines.size(); i++) {
        // bool add_to_cirlce_line = false;
        // filter lines in appropriate length
        float line_len_i = result_lines[i].GetLength();
        if (line_len_i > parameters.circle.maxLineLen || line_len_i < parameters.circle.minLineLen) {
            continue;
        }
        // prepare perpendicular line segment with length of the original one
        LineSegment pls_i = result_lines[i].PerpendicularLineSegment();
        // traverse the following lines in result_lines
        for (size_t j = i + 1; j < result_lines.size(); j++) {
            // filter lines in appropriate length
            float line_len_j = result_lines[j].GetLength();
            if (line_len_j > parameters.circle.maxLineLen || line_len_j < parameters.circle.minLineLen) {
                continue;
            }
            // filter lines within appropriate distance (in robot coord)
            if (dist3D_Segment_to_Segment(result_lines[j], result_lines[i]) > parameters.circle.maxDistBetween2LS) {
                continue;
            }
            // prepare perpendicular line segment with length of the original one
            LineSegment pls_j = result_lines[j].PerpendicularLineSegment();
            // get intersect point of two lines
            cv::Point2d intersect_point;
            if (pls_i.IntersectLineForm(pls_j, intersect_point)) {
                // filter intersect point with appropriate distance from two lines
                float distance1 = result_lines[i].DistanceFromLine(intersect_point);
                float distance2 = result_lines[j].DistanceFromLine(intersect_point);
                if (distance1 < parameters.field_model.center_circle_diameter / 2.0 * parameters.circle.radiusMaxCoef &&
                    distance1 > parameters.field_model.center_circle_diameter / 2.0 * parameters.circle.radiusMinCoef &&
                    distance2 < parameters.field_model.center_circle_diameter / 2.0 * parameters.circle.radiusMaxCoef &&
                    distance2 > parameters.field_model.center_circle_diameter / 2.0 * parameters.circle.radiusMinCoef) {
                    // if this intersect point is valid, push it back to circle points candidates
                    circle_point_candidates.push_back(intersect_point);
                }
            }
        }
    }

    // fuse all circle point candidates
    if (circle_point_candidates.size() >= static_cast<size_t>(parameters.circle.minLineSegmentCount)) {
        cv::Point2f sum;
        // get average center circle point
        for (auto candidates : circle_point_candidates) {
            sum += candidates;
        }
        result_circle_.x = sum.x / circle_point_candidates.size();
        result_circle_.y = sum.y / circle_point_candidates.size();
        ;

        // remove point far from average
        sum.x = 0.0;
        sum.y = 0.0;
        int valid_circle_counter = 0;
        for (auto candidates : circle_point_candidates) {
            if (GetDistance(candidates, result_circle_) < parameters.circle.confiusedDist) {
                sum += candidates;
                valid_circle_counter++;
            } else {
                ROS_DEBUG("remove invalid center point (%f, %f)", candidates.x, candidates.y);
            }
        }
        if (valid_circle_counter > 0) {
            result_circle_.x = sum.x / valid_circle_counter;
            result_circle_.y = sum.y / valid_circle_counter;
            return true;
        }
    }
    return false;
}
} // namespace dvision
