/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-04T12:48:54+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: utils.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-04T12:50:12+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/line_segment.hpp"
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {
#define min_n(a, b) ((a) < (b) ? (a) : (b))
#define max_n(a, b) ((a) > (b) ? (a) : (b))
#define boundry_n(n, a, b)                                                                                                                                                                             \
    {                                                                                                                                                                                                  \
        n = min_n(b, n);                                                                                                                                                                               \
        n = max_n(a, n);                                                                                                                                                                               \
    }

cv::Point2f
RotateCoordinateAxis(const double& alpha, const cv::Point2f& p);

void
RotateCoordinateAxis(const double& alpha, const cv::Point2d& p, cv::Point2d& res);

cv::Point2f
RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha);

cv::Point2f
RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha, const cv::Point2f& pCenter);

void
RotateAroundPoint(const cv::Point2d& pQuery, const double& alpha, cv::Point2d& res);

double
Radian2Degree(const double& r);

double
Degree2Radian(const double& d);

void
LowPass(const cv::Point3f& newrec, cv::Point3f& res, const float& coef);

void
LowPass(const cv::Point2f& newrec, cv::Point2f& res, const float& coef);

void
LowPass(const float& newrec, float& res, const float& coef);

double
GetDistance(const cv::Point2d& p);

float
GetDistance(const cv::Point2f& p);

double
GetDistance(const cv::Point2d& p, const cv::Point2d& p2);

float
GetDistance(const cv::Point2f& p, const cv::Point2f& p2);

float
GetDistance(const cv::Point& p, const cv::Point& p2);

int
Top(const cv::Rect& rec);

int
Bottom(const cv::Rect& rec);

int
Left(const cv::Rect& rec);

int
Right(const cv::Rect& rec);

cv::Scalar
grayWhite();

cv::Scalar
pinkColor();

cv::Scalar
pinkMeloColor();

cv::Scalar
whiteColor();

cv::Scalar
redColor();

cv::Scalar
magentaNormalColor();

cv::Scalar
orangeColor();

cv::Scalar
darkOrangeColor();

cv::Scalar
redMeloColor();

cv::Scalar
greenColor();

cv::Scalar
yellowColor();

cv::Scalar
blueColor();

cv::Scalar
blueMeloColor();

cv::Scalar
blueNormalColor();

cv::Scalar
blueLightColor();

cv::Scalar
blueGrayColor();

cv::Scalar
blackColor();

cv::Scalar
blackGary();

bool
MergeLinesMax(std::vector<LineSegment> resLinesReal,
              const double& maxDegree,
              const double& maxDistance,
              const double& minCollinearLength,
              std::vector<LineSegment>& clusteredLines,
              const cv::Rect& box,
              const bool& useBounding = false);

bool
MergeLinesOnce(std::vector<LineSegment> resLinesReal,
               const double& maxDegree,
               const double& maxDistance,
               const double& minCollinearLength,
               std::vector<LineSegment>& clusteredLines,
               const cv::Rect& box,
               const bool& useBounding = false);

float
dist3D_Segment_to_Segment(const LineSegment& S1, const LineSegment& S2);

// Normalize to [0,360):
double
CorrectAngleDegree360(const double& x);

// Normalize to [-180,180)
double
CorrectAngleDegree180(const double& x);

// Normalize to [0,360):
double
CorrectAngleRadian360(const double& x);

// Normalize to [-180,180)
double
CorrectAngleRadian180(const double& x);

// Normalize to [-180,180)
float
AngleDiffDegree180(const float& first, const float& second);

// Normalize to [-180,180)
float
AngleDiffRadian180(const float& first, const float& second);

cv::Point2f
GetAverage(const cv::Point2f& p0, const cv::Point2f& p1);

cv::Point2f
GetWeightedAverage(const cv::Point2f& p0, const cv::Point2f& p1, const float& w0, const float& w1);

float
GetWeightedAverage(const float& p0, const float& p1, const float& w0, const float& w1);

float
DistanceFromLineSegment(const LineSegment& line, const cv::Point2f& p);

bool
SortFuncDescending(const std::vector<cv::Point>& i, const std::vector<cv::Point>& j);

std::vector<LineSegment>
getOnGlobalCoordinate(const cv::Point3d& robot_pos, const std::vector<LineSegment>& in_lines);

std::vector<cv::Point2f>
getOnGlobalCoordinate(const cv::Point3d& robot_pos, const std::vector<cv::Point2f>& in_points);

cv::Point2f
getOnGlobalCoordinate(const cv::Point3d& robot_pos, const cv::Point2f& in_point);

cv::Point2f
getOnGlobalCoordinate(const cv::Point3d& robot_pos, float x, float y);

cv::Point2f
getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos, float x, float y);

cv::Point2f
getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos, const cv::Point2f& in_point);

cv::Point2f
getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos, const geometry_msgs::Vector3& in_point);

cv::Point2d
getOnGlobalCoordinate(const cv::Point3d& robot_pos, const cv::Point2d& in_point);

cv::Point2d
getOnRobotCoordinate(const cv::Point3d& robot_pos, const cv::Point2d& in_point);

inline float
getAngleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2)
{
    float dot = v1.x * v2.x + v1.y * v2.y;
    float det = v1.x * v2.y - v1.y * v2.x;
    return Radian2Degree(atan2f(det, dot));
}

template<typename Iter, typename Func>
void
find_pairwise(Iter first, Iter last, Func func)
{
    for (; first != last; ++first)
        for (Iter next = std::next(first); next != last; ++next)
            func(*first, *next);
}

} // namespace dvision
