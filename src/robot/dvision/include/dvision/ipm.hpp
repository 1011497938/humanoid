// Created on: May 13, 2017
//     Author: Yujie Yang <meetyyj@gmail.com>
//     Author: Wenxing Mei <mwx37mwx@gmail.com>
// Note: IPM must be inited after DistortionModel initialize
#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include "dvision/parameters.hpp"
#include "dvision/math.hpp"

using dvision::parameters;
using Eigen::MatrixXd;
namespace dvision {
class IPM
{
  public:
    // set camera matrix and extrinsic parameters
    void Init(std::vector<double> extrinsic_para, double undistFx, double undistFy, double undistCx, double undistCy);

    // update yaw and pitch, then first calculate extrinsic and then calculate m_A and m_invA
    void update(double pitch, double yaw);
    void updateDeg(double pitch ,double yaw);

    // use m_A to project a 3D point into image plane
    // Matlab code: projection.m
    cv::Point2d project(double x_real, double y_real, double z_real = 0);

    // use m_invA to inverse project a point on image plane onto the z_real plane
    // normally z_real equals 0 means the object is on the ground
    // when considering the height, such as the center of the ball, z_real can be used
    // Matlab code: calc_xy.m
    cv::Point2d inverseProject(double u, double v, double z_real = 0);

private:
    void calc_extrinsic(double pitch, double yaw);

    Eigen::MatrixXd m_cameraMatrix; // fx fy cx cy
    std::vector<double> m_extrinsic_para; // 16 parameters to calculate extrinsic

    Eigen::MatrixXd m_extrinsic;
    Eigen::MatrixXd m_A; // camera matrix * extrinsic matrix
    Eigen::MatrixXd m_invA; // inverse of m_A;

};
} // namespace dvision
