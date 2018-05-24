/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-04T12:47:06+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: line_segment.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-04T12:49:05+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {

#define DEFFLEQEPSILON 0.001

class LineSegment
{
  public:
    LineSegment(const cv::Point2d p1, const cv::Point2d p2, double _probability = 0.0);
    LineSegment(const cv::Point2d center, double angle, double length, double _probability = 0.0);
    LineSegment(const LineSegment& l);
    LineSegment();
    ~LineSegment();

    // Setter
    void SetProbability(const double& _in);
    void SetDownPoint(const cv::Point2f& alter_down);
    void SetUpPoint(const cv::Point2f& alter_up);

    // Getter
    double GetProbability() const;
    double GetLength() const;
    cv::Point2f GetClosestPointOnLineSegment(cv::Point2f p);
    cv::Point2f GetMiddle();
    cv::Point2f GetUpPoint();
    cv::Point2f GetDownPoint();
    void GetDirection(cv::Point2d& res) const;
    float GetYByX(const float& x) const;
    int GetSide(const cv::Point2d& point) const;
    double GetExteriorAngleDegree(const LineSegment& otherLine) const;
    double GetAbsMinAngleDegree(const LineSegment& otherLine) const;
    bool GetSlope(double& slope);
    double GetRadianFromX();
    double GetDegreeFromX();
    std::vector<LineSegment> GetMidLineSegments(const int& count);
    std::vector<cv::Point2d> GetMidPoints(const int& count, const bool& sortthis = true);

    // Operation
    float DistanceFromLine(const cv::Point2f& p);
    bool Intersect(LineSegment L, cv::Point2d& res);
    bool IntersectLineForm(LineSegment L, cv::Point2d& res);
    LineSegment PerpendicularLineSegment(const double& scale = 1);
    LineSegment PerpendicularLineSegment(const double& len, const cv::Point2d& mid);
    cv::Point2d ExtensionPointDown(const double& len);
    cv::Point2d ExtensionPointUp(const double& len);
    LineSegment ExtensionCordDown(const double& len);
    LineSegment ExtensionCordUp(const double& len);
    LineSegment Scale(const double& _in);
    bool SortbyDistance(const cv::Point2d& a, const cv::Point2d& b);
    bool IsOnThis(const cv::Point2f& ptTest, float flEp = DEFFLEQEPSILON);
    void Clip(cv::Rect boundry);

    // TODO(corenel) make m_P1, m_P2 private
    cv::Point2d P1, P2;

  private:
    double probability_;
    bool Within(const float& fl, const float& flLow, const float& flHi, const float& flEp = DEFFLEQEPSILON);
};

class LinearInterpolator
{
  public:
    explicit LinearInterpolator(LineSegment line);
    LinearInterpolator(const cv::Point2d& p1, const cv::Point2d& p2);
    double Interpolate(const double& x);
    ~LinearInterpolator();

  private:
    LineSegment line_;
};

class LinearBoundaryChecker
{
  public:
    bool checkValidity();
    LinearBoundaryChecker(const LineSegment& _LLow, const LineSegment& _LHigh);
    LinearBoundaryChecker(const double& nearDistance, const double& nearMin, const double& nearMax, const double& farDistance, const double& farMin, const double& farMax);
    bool CheckExtrapolation(const double& distance, const double& value);
    bool GetExtrapolation(const double& distance, double& min, double& max);
    bool CheckInside(const double& distance, const double& value);
    ~LinearBoundaryChecker();

  private:
    LineSegment LLow, LHigh;
};

} // namespace dvision
