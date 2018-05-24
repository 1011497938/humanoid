//
// Created by yyj on 17-6-15.
//

#pragma once

#include "dvision/math.hpp"
#include "dvision/parameters.hpp"
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
namespace dvision {
class Tracker
{
  public:
    bool
    //Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy, double undist_pos_x = parameters.camera.undistCx / 2, double undist_pos_y = parameters.camera.undistCy / 2);
    Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy, double undist_pos_x, double undist_pos_y);

    //球在场地上相对机器人的位置
    //当前的pitch,yaw的值
    // bool Process(double in_ball_field_x, double in_ball_field_y, double in_pitch, double in_yaw);
    bool Process(double in_ball_field_x, double in_ball_field_y);

    double Cal_theta_Asin_Bcos_C(double _a, double _b, double _c, double theta_raw);
    inline double out_pitch()
    {
        return m_out_pitch;
    }
    inline double out_yaw()
    {
        return m_out_yaw;
    }

  private:
    double m_out_pitch;
    double m_out_yaw;
    MatrixXd m_cameraMatrix;
    MatrixXd m_w2p;
    MatrixXd m_p2c;
    MatrixXd m_c2i;
    int undist_center_u;
    int undist_center_v;
    double Xw2p;
    double Yw2p;
    double Zw2p;
    double RXw2p;
    double RYw2p;
    double RZw2p;
    double Xp2c;
    double Yp2c;
    double Zp2c;
    double RXp2c;
    double RYp2c;
    double RZp2c;
    double scaleYaw;
    double scalePitch;
    double biasYaw;
    double biasPitch;
};
}
