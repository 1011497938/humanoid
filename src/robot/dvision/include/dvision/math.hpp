// https://en.wikipedia.org/wiki/Rotation_matrix
#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cmath>

using std::cos;
using std::sin;
using Eigen::MatrixXd;

namespace dvision {

inline MatrixXd rotateX(double r) {
    MatrixXd R(4, 4);
    R << 1, 0, 0, 0,
        0, cos(r), -sin(r), 0,
        0, sin(r), cos(r), 0,
        0,  0, 0, 1;
    return R;
}

inline MatrixXd rotateY(double r) {
    MatrixXd R(4, 4);
    R << cos(r), 0, sin(r), 0,
        0, 1, 0, 0,
        -sin(r), 0, cos(r), 0,
        0, 0, 0, 1;
    return R;
}

inline MatrixXd rotateZ(double r) {
    MatrixXd R(4, 4);
    R << cos(r), -sin(r), 0, 0,
        sin(r), cos(r), 0, 0,
        0,      0, 1, 0,
        0,      0, 0, 1;
    return R;
}

inline MatrixXd dtranslate(double x, double y, double z) {
    MatrixXd T(4, 4);
    T << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T;
}

}