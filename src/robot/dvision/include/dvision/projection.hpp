// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// TODO(MWX):
// Version 1: only consider camera yaw and pitch

#pragma once
#include "dvision/distortionModel.hpp"
#include "dvision/ipm.hpp"
#include "dvision/utils.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {
class Projection
{
  public:
    explicit Projection();
    ~Projection();
    void init(ros::NodeHandle*);

  public:
    void updateExtrinsic(double pitch, double yaw);

    inline bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res)
    {
        return m_dist.undistort(points, res);
    }

    inline bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res)
    {
        return m_dist.undistort(points, res);
    }

    inline bool distort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res)
    {
        m_dist.distort(points, res);
        return true;
    }

    inline bool distortP(const std::vector<cv::Point>& points, std::vector<cv::Point>& res)
    {
        m_dist.distortP(points, res);
        return true;
    }

    bool getOnImageCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& res_points);
    bool getOnRealCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res_points, float z_real = 0);

    // lines
    bool getOnImageCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines);
    bool getOnRealCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines, float z_real = 0);

    // single point
    bool getOnImageCoordinate(const cv::Point2f& point, cv::Point& res_point);
    bool getOnRealCoordinate(const cv::Point& point, cv::Point2f& res_point, float z_real = 0);

    // rotate
    std::vector<cv::Point2f> RotateTowardHeading(const std::vector<cv::Point2f>& in);
    cv::Point2d RotateTowardHeading(const cv::Point2d& in);
    cv::Point2f RotateTowardHeading(const cv::Point2f& in);
    std::vector<LineSegment> RotateTowardHeading(const std::vector<LineSegment>& in);
    bool CalcHeadingOffset(std::vector<LineSegment>& result_lines, bool circle_detected, const cv::Point2d& result_circle, const std::vector<cv::Point2f>& goal_position);
    bool UpdateHeadingOffset(double &in_yaw);
    inline double GetHeading()
    {
        // In Radian
        // if (std::abs(Radian2Degree(m_heading_offset)) > 90) {
        //     ROS_WARN("Heading offset flip prevented!");
        //     m_heading_offset = 0;
        // }
        // TODO(corenel) how to get heading offset?
        return CorrectAngleRadian360(0 + m_heading_offset);
        // return CorrectAngleRadian360(headingData.heading + m_heading_offset);
    }

    inline DistortionModel* dist()
    {
        return &m_dist;
    }
    void SetHeadingOffsetBias(double bias);

  private:
    void init();

  private:
    DistortionModel m_dist;
    IPM m_ipm;
    // VERSION 1, use only yaw and pitch, 2017/5/29
    double m_yaw;
    double m_pitch;
    double m_heading_offset; // in radian
    double m_heading_offset_bias; // in radian
    Eigen::MatrixXd m_extrinsic;

    cv::Mat homoImgToReal; // real = homo * img
    cv::Mat homoRealToImg; // img = homo * real
};
} // namespace dvision
