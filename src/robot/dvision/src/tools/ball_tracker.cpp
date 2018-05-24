//
// Created by yyj on 17-6-15.
//

#include "dvision/ball_tracker.hpp"

namespace dvision {
bool
Tracker::Init(std::vector<double> extrinsic_para, double fx, double fy, double cx, double cy, double undist_pos_x, double undist_pos_y)
{
    undist_center_u = undist_pos_x;
    undist_center_v = undist_pos_y;

    Xw2p = extrinsic_para[0];
    Yw2p = extrinsic_para[1];
    Zw2p = extrinsic_para[2];

    RXw2p = extrinsic_para[3];
    RYw2p = extrinsic_para[4];
    RZw2p = extrinsic_para[5];
    Xp2c = extrinsic_para[6];
    Yp2c = extrinsic_para[7];
    Zp2c = extrinsic_para[8];

    RXp2c = extrinsic_para[9];
    RYp2c = extrinsic_para[10];
    RZp2c = extrinsic_para[11];

    scaleYaw = extrinsic_para[12];
    scalePitch = extrinsic_para[13];
    biasYaw = extrinsic_para[14];
    biasPitch = extrinsic_para[15];

    MatrixXd c2i(4, 4); // camera to image
    c2i << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    m_c2i = c2i;

    MatrixXd cameraMatrix(3, 4);
    cameraMatrix << fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0;

    m_cameraMatrix = cameraMatrix;

    m_w2p = rotateZ(RZw2p) * rotateY(RYw2p) * rotateX(RXw2p) * dtranslate(-Xw2p, -Yw2p, -Zw2p);

    m_p2c = rotateZ(RZp2c) * rotateY(RYp2c) * rotateX(RXp2c) * dtranslate(-Xp2c, -Yp2c, -Zp2c);

    return true;
}

// bool BalllTracker::Process(double in_ball_field_x, double in_ball_field_y,
//                                double in_pitch, double in_yaw) {
//
//
//     in_pitch = (in_pitch + biasPitch) * scalePitch;
//     in_yaw = (in_yaw + biasYaw) * scaleYaw;
//
//
//     // m_out_pitch = -in_pitch;
//     // m_out_yaw = -in_yaw;
//
//     MatrixXd field_xy0(4, 1);
//     field_xy0 << in_ball_field_x,
//                  in_ball_field_y,
//                  0,
//                  1;
//
//
//
//
//     double approx_yaw;
//     if(in_ball_field_x < 0.000001 && in_ball_field_x > 0.000001){
//       if(in_ball_field_y > 0){
//         approx_yaw = M_PI / 2;
//       } else {
//         approx_yaw = -M_PI / 2;
//       }
//     } else if(in_ball_field_x > 0){
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x);
//     } else if(in_ball_field_y > 0){
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x) + M_PI;
//     } else {
//       approx_yaw = atan(in_ball_field_y / in_ball_field_x) - M_PI;
//     }
//     // cout << "approx_yaw: " << approx_yaw / M_PI * 180 << endl;
//     int repeat_times = 2;
//     for (int i = 0; i < repeat_times; ++i) {
//       //cal yaw
//       MatrixXd m_y = m_cameraMatrix * m_c2i * m_p2c * rotateY(0);
//       // MatrixXd m_y = m_cameraMatrix * m_c2i * m_p2c * rotateY(m_out_pitch);
//       MatrixXd n_y = m_w2p * field_xy0;
//       double A_y = -undist_center_u * m_y(2, 0) * n_y(1, 0) + undist_center_u * m_y(2, 1) * n_y(0, 0) + m_y(0, 0) * n_y(1, 0) - m_y(0, 1) * n_y(0, 0);
//       double B_y = undist_center_u * m_y(2, 0) * n_y(0, 0) + undist_center_u * m_y(2, 1) * n_y(1, 0) - m_y(0, 0) * n_y(0, 0) - m_y(0, 1) * n_y(1, 0);
//       double C_y = undist_center_u * m_y(2, 2) * n_y(2, 0) + undist_center_u * m_y(2, 3) * n_y(3, 0) - m_y(0, 2) * n_y(2, 0) - m_y(0, 3) * n_y(3, 0);
//       m_out_yaw = Cal_theta_Asin_Bcos_C(A_y, B_y, C_y, -approx_yaw);
//       //
//       // MatrixXd s_uv = m_y * rotateZ(m_out_yaw) * n_y;
//       // cout << "-----------" << endl;
//       // cout << "第" << i << "次调整 左右:" << endl;
//       // cout << "u: " << s_uv(0, 0)/s_uv(2, 0) << endl;
//       // cout << "v: " << s_uv(1, 0)/s_uv(2, 0) << endl;
//       // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
//       // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
//       // cout << "-----------" << endl;
//       //cal pitch
//         MatrixXd m_p = m_cameraMatrix * m_c2i * m_p2c;
//         MatrixXd n_p = rotateZ(m_out_yaw) * m_w2p * field_xy0;
//
//         double A_p = undist_center_v * n_p(2, 0) * m_p(2, 0) - undist_center_v * n_p(0, 0) * m_p(2, 2) - n_p(2, 0) * m_p(1, 0) + n_p(0, 0) * m_p(1, 2);
//         double B_p = undist_center_v * n_p(0, 0) * m_p(2, 0) + undist_center_v * n_p(2, 0) * m_p(2, 2) - n_p(0, 0) * m_p(1, 0) - n_p(2, 0) * m_p(1, 2);
//         double C_p = undist_center_v * n_p(1, 0) * m_p(2, 1) + undist_center_v * n_p(3, 0) * m_p(2, 3) - n_p(1, 0) * m_p(1, 1) - n_p(3, 0) * m_p(1, 3);
//         // m_out_pitch = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, m_out_pitch);
//         m_out_pitch = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, 0);
//
//         // MatrixXd s_v = m_p * rotateY(m_out_pitch) * n_p;
//         // cout << "-----------" << endl;
//         // cout << "第" << i << "次调整 俯仰:" << endl;
//         // cout << "u: " << s_v(0, 0)/s_v(2, 0) << endl;
//         // cout << "v: " << s_v(1, 0)/s_v(2, 0) << endl;
//         // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
//         // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
//         // cout << "-----------" << endl;
//
//
//     }
//
//     m_out_pitch = - m_out_pitch / scalePitch - biasPitch;
//     m_out_yaw = - m_out_yaw / scaleYaw - biasYaw;
//     cout << "final pitch: " << m_out_pitch / M_PI * 180 << endl;
//     cout << "final yaw: " << m_out_yaw / M_PI * 180 << endl;
//   //  if (m_out_pitch >= 0 && m_out_pitch < M_PI / 2
//   //     && m_out_yaw >= -M_PI / 2 && m_out_yaw <= M_PI/2){
//   //       return true;
//   //  }
//   //  else{
//   //    return false;
//   //  }
//
//     return true;
// }

bool
Tracker::Process(double in_ball_field_x, double in_ball_field_y)
{
    if(std::isnan(in_ball_field_x) || std::isnan(in_ball_field_y)) {
        m_out_yaw = 0.0;
        m_out_pitch = 0.0;
        return true;
    }

    MatrixXd field_xy0(4, 1);
    field_xy0 << in_ball_field_x, in_ball_field_y, 0, 1;

    double approx_yaw;
    if (in_ball_field_x < 0.000001 && in_ball_field_x > 0.000001) {
        if (in_ball_field_y > 0) {
            approx_yaw = M_PI / 2;
        } else {
            approx_yaw = -M_PI / 2;
        }
    } else if (in_ball_field_x > 0) {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x);
    } else if (in_ball_field_y > 0) {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x) + M_PI;
    } else {
        approx_yaw = atan(in_ball_field_y / in_ball_field_x) - M_PI;
    }
    //  cout << "x: " << in_ball_field_x << endl;
    //  cout << "y: " << in_ball_field_y << endl;

    // cout << "approx_yaw: " << approx_yaw / M_PI * 180 << endl;
    m_out_yaw = -approx_yaw;
    m_out_pitch = 0;
    int repeat_times = 2;
    for (int i = 0; i < repeat_times; ++i) {
        // cal yaw
        MatrixXd m_y = m_cameraMatrix * m_c2i * m_p2c * rotateY(m_out_pitch);
        MatrixXd n_y = m_w2p * field_xy0;
        double A_y = -undist_center_u * m_y(2, 0) * n_y(1, 0) + undist_center_u * m_y(2, 1) * n_y(0, 0) + m_y(0, 0) * n_y(1, 0) - m_y(0, 1) * n_y(0, 0);
        double B_y = undist_center_u * m_y(2, 0) * n_y(0, 0) + undist_center_u * m_y(2, 1) * n_y(1, 0) - m_y(0, 0) * n_y(0, 0) - m_y(0, 1) * n_y(1, 0);
        double C_y = undist_center_u * m_y(2, 2) * n_y(2, 0) + undist_center_u * m_y(2, 3) * n_y(3, 0) - m_y(0, 2) * n_y(2, 0) - m_y(0, 3) * n_y(3, 0);
        m_out_yaw = Cal_theta_Asin_Bcos_C(A_y, B_y, C_y, m_out_yaw);
        //
        // MatrixXd s_uv = m_y * rotateZ(m_out_yaw) * n_y;
        // cout << "-----------" << endl;
        // cout << "第" << i << "次调整 左右:" << endl;
        // cout << "u: " << s_uv(0, 0)/s_uv(2, 0) << endl;
        // cout << "v: " << s_uv(1, 0)/s_uv(2, 0) << endl;
        // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
        // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
        // cout << "-----------" << endl;
        // cal pitch
        MatrixXd m_p = m_cameraMatrix * m_c2i * m_p2c;
        MatrixXd n_p = rotateZ(m_out_yaw) * m_w2p * field_xy0;

        double A_p = undist_center_v * n_p(2, 0) * m_p(2, 0) - undist_center_v * n_p(0, 0) * m_p(2, 2) - n_p(2, 0) * m_p(1, 0) + n_p(0, 0) * m_p(1, 2);
        double B_p = undist_center_v * n_p(0, 0) * m_p(2, 0) + undist_center_v * n_p(2, 0) * m_p(2, 2) - n_p(0, 0) * m_p(1, 0) - n_p(2, 0) * m_p(1, 2);
        double C_p = undist_center_v * n_p(1, 0) * m_p(2, 1) + undist_center_v * n_p(3, 0) * m_p(2, 3) - n_p(1, 0) * m_p(1, 1) - n_p(3, 0) * m_p(1, 3);
        m_out_pitch = Cal_theta_Asin_Bcos_C(A_p, B_p, C_p, m_out_pitch);

        // MatrixXd s_v = m_p * rotateY(m_out_pitch) * n_p;
        // cout << "-----------" << endl;
        // cout << "第" << i << "次调整 俯仰:" << endl;
        // cout << "u: " << s_v(0, 0)/s_v(2, 0) << endl;
        // cout << "v: " << s_v(1, 0)/s_v(2, 0) << endl;
        // cout << "pitch: " << m_out_pitch / M_PI * 180 << endl;
        // cout << "yaw: " << m_out_yaw / M_PI * 180 << endl;
        // cout << "-----------" << endl;
    }

    m_out_pitch = -m_out_pitch / scalePitch - biasPitch;
    m_out_yaw = -m_out_yaw / scaleYaw - biasYaw;
    // cout << "final pitch: " << m_out_pitch / M_PI * 180 << endl;
    // cout << "final yaw: " << m_out_yaw / M_PI * 180 << endl;
    //  if (m_out_pitch >= 0 && m_out_pitch < M_PI / 2
    //     && m_out_yaw >= -M_PI / 2 && m_out_yaw <= M_PI/2){
    //       return true;
    //  }
    //  else{
    //    return false;
    //  }

    return true;
}

double
Tracker::Cal_theta_Asin_Bcos_C(double _a, double _b, double _c, double theta_raw)
{
    double sin_theta_1 = (-2 * _a * _c + sqrt(4 * _a * _a * _c * _c - 4 * (_a * _a + _b * _b) * (_c * _c - _b * _b))) / (2 * (_a * _a + _b * _b));
    double sin_theta_2 = (-2 * _a * _c - sqrt(4 * _a * _a * _c * _c - 4 * (_a * _a + _b * _b) * (_c * _c - _b * _b))) / (2 * (_a * _a + _b * _b));
    double res, res1 = 999, res2 = 999;
    if (abs(sin_theta_1) <= 1) {
        res1 = asin(sin_theta_1);
        if (res1 >= 0) {
            res1 = (abs(_a * sin(res1) + _b * cos(res1) + _c) < abs(_a * sin(M_PI - res1) + _b * cos(M_PI - res1) + _c)) ? res1 : M_PI - res1;
        } else {
            res1 = (abs(_a * sin(res1) + _b * cos(res1) + _c) < abs(_a * sin(-M_PI - res1) + _b * cos(-M_PI - res1) + _c)) ? res1 : -M_PI - res1;
        }
        // cout << "res1: " << res1 / M_PI * 180 << endl;
    }
    if (abs(sin_theta_2) <= 1) {
        res2 = asin(sin_theta_2);
        if (res2 >= 0) {
            res2 = (abs(_a * sin(res2) + _b * cos(res2) + _c) < abs(_a * sin(M_PI - res2) + _b * cos(M_PI - res2) + _c)) ? res2 : M_PI - res2;
        } else {
            res2 = (abs(_a * sin(res2) + _b * cos(res2) + _c) < abs(_a * sin(-M_PI - res2) + _b * cos(-M_PI - res2) + _c)) ? res2 : -M_PI - res2;
        }
        // cout << "res2: " << res2 / M_PI * 180 << endl;
    }
    //    cout << _a * sin(res1) + _b * cos(res1) + _c << endl;
    //    cout << _a * sin(res2) + _b * cos(res2) + _c << endl;
    res = (abs(theta_raw - res1) < abs(abs(theta_raw - res1) - 2 * M_PI) ? abs(theta_raw - res1) : abs(abs(theta_raw - res1) - 2 * M_PI)) <
              (abs(theta_raw - res2) < abs(abs(theta_raw - res2) - 2 * M_PI) ? abs(theta_raw - res2) : abs(abs(theta_raw - res2) - 2 * M_PI))
            ? res1
            : res2;
    // res = (abs(theta_raw - res1) < abs(theta_raw - res2)) ? res1 : res2;
    // cout << "res: " << res / M_PI * 180 << endl;
    //    cout << _a * sin(res) + _b * cos(res) + _c << endl;
    return res;
}
}