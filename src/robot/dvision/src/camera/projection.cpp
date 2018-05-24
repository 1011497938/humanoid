#include "dvision/projection.hpp"
#include "dvision/parameters.hpp"
#include <numeric>
#include <dvision/timer.hpp>
using namespace std;
using namespace cv;

namespace dvision {
Projection::Projection()
{
}

void
Projection::init(ros::NodeHandle* nh)
{
    m_dist.init();
    m_ipm.Init(parameters.camera.extrinsic_para, parameters.camera.fx, parameters.camera.fy, parameters.camera.undistCx, parameters.camera.undistCy);
}

Projection::~Projection() = default;

void
Projection::updateExtrinsic(double pitch, double yaw)
{
    if(-90 <= pitch && pitch <= 90 && -180 <= yaw && yaw <= 180) {
        Timer t;
        m_ipm.updateDeg(pitch, yaw);
        ROS_DEBUG("updateExtrinsic used: %lf ms", t.elapsedMsec());
    } else {
        throw std::runtime_error("Pitch and yaw error");
    }

}

// Single point

bool
Projection::getOnImageCoordinate(const Point2f& point, Point& resPoint)
{
    Point2d undist_point = m_ipm.project(point.x, point.y);
    // resPoint = m_dist.fuck(undist_point.x, undist_point.y);
    m_dist.distortP(undist_point, resPoint);
    return true;
}

bool
Projection::getOnRealCoordinate(const Point& point, Point2f& resPoint, float z_real)
{
    Point undist_point = m_dist.undistort(point.x, point.y);
    resPoint = m_ipm.inverseProject(undist_point.x, undist_point.y, z_real);
    return true;
}

// Points

bool
Projection::getOnImageCoordinate(const vector<Point2f>& points, vector<Point>& resPoints)
{
    resPoints.resize(points.size());
    for (uint32_t i = 0; i < points.size(); ++i) {
        getOnImageCoordinate(points[i], resPoints[i]);
    }
    return true;
}

bool
Projection::getOnRealCoordinate(const vector<Point>& points, vector<Point2f>& resPoints, float z_real)
{
    resPoints.resize(points.size());
    for (uint32_t i = 0; i < points.size(); ++i) {
        getOnRealCoordinate(points[i], resPoints[i], z_real);
    }
    return true;
}

// Lines
bool
Projection::getOnImageCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines)
{
    res_lines.resize(lines.size());
    Point tmp;
    for (uint32_t i = 0; i < lines.size(); ++i) {
        getOnImageCoordinate(lines[i].P1, tmp);
        res_lines[i].P1.x = tmp.x;
        res_lines[i].P1.y = tmp.y;

        getOnImageCoordinate(lines[i].P2, tmp);
        res_lines[i].P2.x = tmp.x;
        res_lines[i].P2.y = tmp.y;

        res_lines[i].SetProbability(lines[i].GetProbability());
    }
    return true;
}

bool
Projection::getOnRealCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines, float z_real)
{
    res_lines.resize(lines.size());
    Point2f tmp;
    for (uint32_t i = 0; i < lines.size(); ++i) {
        getOnRealCoordinate(lines[i].P1, tmp, z_real);
        res_lines[i].P1.x = tmp.x;
        res_lines[i].P1.y = tmp.y;

        getOnRealCoordinate(lines[i].P2, tmp, z_real);
        res_lines[i].P2.x = tmp.x;
        res_lines[i].P2.y = tmp.y;

        res_lines[i].SetProbability(lines[i].GetProbability());
    }
    return true;
}

std::vector<cv::Point2f>
Projection::RotateTowardHeading(const std::vector<cv::Point2f>& in)
{
    std::vector<cv::Point2f> out(in.size());
    for (size_t i = 0; i < in.size(); i++) {
        out[i] = RotateTowardHeading(in[i]);
    }
    return out;
}

cv::Point2d
Projection::RotateTowardHeading(const cv::Point2d& in)
{
    return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

cv::Point2f
Projection::RotateTowardHeading(const cv::Point2f& in)
{
    return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

std::vector<LineSegment>
Projection::RotateTowardHeading(const std::vector<LineSegment>& in)
{
    std::vector<LineSegment> out(in.size());
    for (size_t i = 0; i < in.size(); i++) {
        out[i] = LineSegment(RotateTowardHeading(in[i].P1), RotateTowardHeading(in[i].P2), in[i].GetProbability());
    }
    return out;
}

bool
Projection::CalcHeadingOffset(std::vector<LineSegment>& result_lines, bool circle_detected, const Point2d& result_circle, const std::vector<cv::Point2f>& goal_position)
{
    LineSegment VerLine(cv::Point(0, -10), cv::Point(0, 10));
    std::vector<double> heading_offset_vec;
    for (auto line_seg : result_lines) {
        if (line_seg.GetLength() > parameters.line_classifier.minLineLen) {
            double angle_diff_ver = line_seg.GetExteriorAngleDegree(VerLine);
            if (angle_diff_ver < -90)
                angle_diff_ver += 180;
            if (angle_diff_ver > 90)
                angle_diff_ver += -180;

            if (circle_detected && DistanceFromLineSegment(line_seg, result_circle) < 50) {
                heading_offset_vec.push_back(angle_diff_ver);
                // cout << "CalcHeadingOffset: HorCenter angle " <<
                // heading_offset_vec.back()
                //      << endl;
            }
            if (goal_position.size() == 2 && line_seg.DistanceFromLine(goal_position[0]) < parameters.line_classifier.maxDistBothGoal &&
                line_seg.DistanceFromLine(goal_position[1]) < parameters.line_classifier.maxDistBothGoal) {
                heading_offset_vec.push_back(angle_diff_ver);
                // cout << "CalcHeadingOffset: goal line angle " <<
                // heading_offset_vec.back()
                //      << endl;
            }
        }
    }
    if (!heading_offset_vec.empty()) {
        double heading_offset_avg = accumulate(heading_offset_vec.begin(), heading_offset_vec.end(), 0.0) / heading_offset_vec.size();
        // remove angle far from average
        int valid_counter = 0;
        double sum = 0;
        for (size_t counter = 0; counter < heading_offset_vec.size(); counter++) {
            if (std::abs(heading_offset_vec[counter] - heading_offset_avg) < 15) {
                sum += heading_offset_vec[counter];
                valid_counter++;
            }
        }
        if (valid_counter > 0) {
            heading_offset_avg = sum / valid_counter;
            // cout << "CalcHeadingOffset: " << heading_offset_avg << endl;
            m_heading_offset = M_PI / 180 * heading_offset_avg;
        }
    }
    return true;
}
bool
Projection::UpdateHeadingOffset(double &in_yaw)
{
    // std:: cout << "in_yaw raw      : " << *in_yaw / M_PI * 180 << std::endl;
    in_yaw += m_heading_offset_bias;
    m_heading_offset_bias = 0;
    m_heading_offset = in_yaw;
    // std:: cout << "in_yaw corrected: " << *in_yaw / M_PI * 180 << std::endl;
    return true;
}
void
Projection::SetHeadingOffsetBias(double bias){
    m_heading_offset_bias = bias * M_PI / 180;
}


} // namespace dvision
