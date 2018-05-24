#include "dmotion/GaitStateSupportLib/kalman_filter.hpp"

#include <cmath>
#include <iostream>
#include <memory>

namespace KALMAN {
static const ddouble_t c_Q = 1; // 1
static const ddouble_t R = 5000; // 1000
// static const ddouble_t a1 = 3.5;
// static const ddouble_t a2 = 9;
// static const ddouble_t a3 = 0.5;
// static const ddouble_t k1 = 1000;
// static const ddouble_t k2 = 400;
static const ddouble_t c_g = 9.8;
ddouble_t T;
}
using namespace KALMAN;

KalmanFilter::KalmanFilter()
{
    int i, j;
    for (i = 0; i < 3; i++) {
        est_last[i] = 0;
        for (j = 0; j < 3; j++) {
            est_p[i][j] = 0;
        }
    }
    est_last[2] = 1;
}

void
KalmanFilter::estimate(ddouble_t wx, ddouble_t wy, ddouble_t wz, ddouble_t ax, ddouble_t ay, ddouble_t az, ddouble_t time)
{
    KALMAN::T = time;

    int i, j, k;
    ddouble_t est[3];
    ddouble_t pre[3];
    ddouble_t pre_p[3][3];

    ddouble_t H[4][3];
    ddouble_t S[4][4];
    ddouble_t Kg[3][4];
    ddouble_t temp1[3][3];
    ddouble_t temp2[4][3];
    ddouble_t temp3[3][4];
    ddouble_t temp4[4];
    ddouble_t det;
    ddouble_t inverse[4][4];

    wx = wx * 1.0 / 180.0 * 3.1415926;
    wy = wy * 1.0 / 180.0 * 3.1415926;
    wz = wz * 1.0 / 180.0 * 3.1415926;

    A[0][0] = (T * T * wy * wy + T * T * wz * wz) * (T * T * wy * wy + T * T * wz * wz) / 24 - (T * T * wy * wy) / 2 - (T * T * wz * wz) / 2 + (T * T * T * T * wx * wx * wy * wy) / 24 +
              (T * T * T * T * wx * wx * wz * wz) / 24 +
              (T * wy * (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wy * wz)) / 120 -
              (T * wz * (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz) + T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wz * wz)) / 120 + 1;
    A[0][1] = T * wz + (T * T * wx * wy) / 2 - (T * wz * (T * T * wx * wx + T * T * wz * wz)) / 6 - (T * T * T * wy * wy * wz) / 6 +
              (T * wz * ((T * T * wx * wx + T * T * wz * wz) * (T * T * wx * wx + T * T * wz * wz) + T * T * T * T * wx * wx * wy * wy + T * T * T * T * wy * wy * wz * wz)) / 120 +
              (T * wy * (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz) - T * T * T * T * wx * wx * wy * wz)) / 120 -
              (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz)) / 24 - (T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wy * wz * wz) / 24;
    A[0][2] = (T * T * wx * wz) / 2 - T * wy + (T * wy * (T * T * wx * wx + T * T * wy * wy)) / 6 + (T * T * T * wy * wz * wz) / 6 -
              (T * wy * ((T * T * wx * wx + T * T * wy * wy) * (T * T * wx * wx + T * T * wy * wy) + T * T * T * T * wx * wx * wz * wz + T * T * T * T * wy * wy * wz * wz)) / 120 -
              (T * wz * (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz) - T * T * T * T * wx * wx * wy * wz)) / 120 -
              (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy)) / 24 - (T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wy * wy * wz) / 24;
    A[1][0] = (T * T * wx * wy) / 2 - T * wz + (T * wz * (T * T * wy * wy + T * T * wz * wz)) / 6 + (T * T * T * wx * wx * wz) / 6 -
              (T * wz * ((T * T * wy * wy + T * T * wz * wz) * (T * T * wy * wy + T * T * wz * wz) + T * T * T * T * wx * wx * wy * wy + T * T * T * T * wx * wx * wz * wz)) / 120 -
              (T * wx * (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wy * wz)) / 120 -
              (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz)) / 24 - (T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wy * wz * wz) / 24;
    A[1][1] = (T * T * wx * wx + T * T * wz * wz) * (T * T * wx * wx + T * T * wz * wz) / 24 - (T * T * wx * wx) / 2 - (T * T * wz * wz) / 2 + (T * T * T * T * wx * wx * wy * wy) / 24 +
              (T * T * T * T * wy * wy * wz * wz) / 24 -
              (T * wx * (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz) - T * T * T * T * wx * wx * wy * wz)) / 120 +
              (T * wz * (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz) + T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wz * wz)) / 120 + 1;
    A[1][2] = T * wx + (T * T * wy * wz) / 2 - (T * wx * (T * T * wx * wx + T * T * wy * wy)) / 6 - (T * T * T * wx * wz * wz) / 6 +
              (T * wx * ((T * T * wx * wx + T * T * wy * wy) * (T * T * wx * wx + T * T * wy * wy) + T * T * T * T * wx * wx * wz * wz + T * T * T * T * wy * wy * wz * wz)) / 120 +
              (T * wz * (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wy * wz)) / 120 -
              (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy)) / 24 - (T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wx * wy * wz) / 24;
    A[2][0] = T * wy + (T * T * wx * wz) / 2 - (T * wy * (T * T * wy * wy + T * T * wz * wz)) / 6 - (T * T * T * wx * wx * wy) / 6 +
              (T * wy * ((T * T * wy * wy + T * T * wz * wz) * (T * T * wy * wy + T * T * wz * wz) + T * T * T * T * wx * wx * wy * wy + T * T * T * T * wx * wx * wz * wz)) / 120 +
              (T * wx * (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz) + T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wz * wz)) / 120 -
              (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy)) / 24 - (T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wy * wy * wz) / 24;
    A[2][1] = (T * T * wy * wz) / 2 - T * wx + (T * wx * (T * T * wx * wx + T * T * wz * wz)) / 6 + (T * T * T * wx * wy * wy) / 6 -
              (T * wx * ((T * T * wx * wx + T * T * wz * wz) * (T * T * wx * wx + T * T * wz * wz) + T * T * T * T * wx * wx * wy * wy + T * T * T * T * wy * wy * wz * wz)) / 120 -
              (T * wy * (T * T * wx * wy * (T * T * wx * wx + T * T * wz * wz) + T * T * wx * wy * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wz * wz)) / 120 -
              (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy)) / 24 - (T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz)) / 24 + (T * T * T * T * wx * wx * wy * wz) / 24;
    A[2][2] = (T * T * wx * wx + T * T * wy * wy) * (T * T * wx * wx + T * T * wy * wy) / 24 - (T * T * wx * wx) / 2 - (T * T * wy * wy) / 2 + (T * T * T * T * wx * wx * wz * wz) / 24 +
              (T * T * T * T * wy * wy * wz * wz) / 24 +
              (T * wx * (T * T * wy * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wy * wz * (T * T * wx * wx + T * T * wz * wz) - T * T * T * T * wx * wx * wy * wz)) / 120 -
              (T * wy * (T * T * wx * wz * (T * T * wx * wx + T * T * wy * wy) + T * T * wx * wz * (T * T * wy * wy + T * T * wz * wz) - T * T * T * T * wx * wy * wy * wz)) / 120 + 1;

    //公式1
    for (i = 0; i < 3; i++) {
        pre[i] = 0;
        for (j = 0; j < 3; j++)
            pre[i] += A[i][j] * est_last[j];
    }

    //公式2
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            temp1[i][j] = 0;
            for (k = 0; k < 3; k++)
                temp1[i][j] += A[i][k] * est_p[k][j];
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            pre_p[i][j] = 0;
            for (k = 0; k < 3; k++)
                pre_p[i][j] += temp1[i][k] * A[j][k];
        }
    }
    pre_p[0][0] += c_Q;
    pre_p[1][1] += c_Q;
    pre_p[2][2] += c_Q;

    //公式4
    H[0][0] = -c_g;
    H[0][1] = 0;
    H[0][2] = 0;
    H[1][0] = 0;
    H[1][1] = -c_g;
    H[1][2] = 0;
    H[2][0] = 0;
    H[2][1] = 0;
    H[2][2] = -c_g;
    H[3][0] = 2 * pre[0];
    H[3][1] = 2 * pre[1];
    H[3][2] = 2 * pre[2];

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++) {
            temp2[i][j] = 0;
            for (k = 0; k < 3; k++)
                temp2[i][j] += H[i][k] * pre_p[k][j];
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            S[i][j] = 0;
            for (k = 0; k < 3; k++)
                S[i][j] += temp2[i][k] * H[j][k];
        }
    }
    S[0][0] += R;
    S[1][1] += R;
    S[2][2] += R;
    S[3][3] += R;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            temp3[i][j] = 0;
            for (k = 0; k < 3; k++)
                temp3[i][j] += pre_p[i][k] * H[j][k];
        }
    }

    det = S[0][0] * S[1][1] * S[2][2] * S[3][3] - S[0][0] * S[1][1] * S[2][3] * S[3][2] - S[0][0] * S[1][2] * S[2][1] * S[3][3] + S[0][0] * S[1][2] * S[2][3] * S[3][1] +
          S[0][0] * S[1][3] * S[2][1] * S[3][2] - S[0][0] * S[1][3] * S[2][2] * S[3][1] - S[0][1] * S[1][0] * S[2][2] * S[3][3] + S[0][1] * S[1][0] * S[2][3] * S[3][2] +
          S[0][1] * S[1][2] * S[2][0] * S[3][3] - S[0][1] * S[1][2] * S[2][3] * S[3][0] - S[0][1] * S[1][3] * S[2][0] * S[3][2] + S[0][1] * S[1][3] * S[2][2] * S[3][0] +
          S[0][2] * S[1][0] * S[2][1] * S[3][3] - S[0][2] * S[1][0] * S[2][3] * S[3][1] - S[0][2] * S[1][1] * S[2][0] * S[3][3] + S[0][2] * S[1][1] * S[2][3] * S[3][0] +
          S[0][2] * S[1][3] * S[2][0] * S[3][1] - S[0][2] * S[1][3] * S[2][1] * S[3][0] - S[0][3] * S[1][0] * S[2][1] * S[3][2] + S[0][3] * S[1][0] * S[2][2] * S[3][1] +
          S[0][3] * S[1][1] * S[2][0] * S[3][2] - S[0][3] * S[1][1] * S[2][2] * S[3][0] - S[0][3] * S[1][2] * S[2][0] * S[3][1] + S[0][3] * S[1][2] * S[2][1] * S[3][0];
    inverse[0][0] =
      (S[1][1] * S[2][2] * S[3][3] - S[1][1] * S[2][3] * S[3][2] - S[1][2] * S[2][1] * S[3][3] + S[1][2] * S[2][3] * S[3][1] + S[1][3] * S[2][1] * S[3][2] - S[1][3] * S[2][2] * S[3][1]) / det;
    inverse[0][1] =
      (S[1][0] * S[2][3] * S[3][2] - S[1][0] * S[2][2] * S[3][3] + S[1][2] * S[2][0] * S[3][3] - S[1][2] * S[2][3] * S[3][0] - S[1][3] * S[2][0] * S[3][2] + S[1][3] * S[2][2] * S[3][0]) / det;
    inverse[0][2] =
      (S[1][0] * S[2][1] * S[3][3] - S[1][0] * S[2][3] * S[3][1] - S[1][1] * S[2][0] * S[3][3] + S[1][1] * S[2][3] * S[3][0] + S[1][3] * S[2][0] * S[3][1] - S[1][3] * S[2][1] * S[3][0]) / det;
    inverse[0][3] =
      (S[1][0] * S[2][2] * S[3][1] - S[1][0] * S[2][1] * S[3][2] + S[1][1] * S[2][0] * S[3][2] - S[1][1] * S[2][2] * S[3][0] - S[1][2] * S[2][0] * S[3][1] + S[1][2] * S[2][1] * S[3][0]) / det;
    inverse[1][0] =
      (S[0][1] * S[2][3] * S[3][2] - S[0][1] * S[2][2] * S[3][3] + S[0][2] * S[2][1] * S[3][3] - S[0][2] * S[2][3] * S[3][1] - S[0][3] * S[2][1] * S[3][2] + S[0][3] * S[2][2] * S[3][1]) / det;
    inverse[1][1] =
      (S[0][0] * S[2][2] * S[3][3] - S[0][0] * S[2][3] * S[3][2] - S[0][2] * S[2][0] * S[3][3] + S[0][2] * S[2][3] * S[3][0] + S[0][3] * S[2][0] * S[3][2] - S[0][3] * S[2][2] * S[3][0]) / det;
    inverse[1][2] =
      (S[0][0] * S[2][3] * S[3][1] - S[0][0] * S[2][1] * S[3][3] + S[0][1] * S[2][0] * S[3][3] - S[0][1] * S[2][3] * S[3][0] - S[0][3] * S[2][0] * S[3][1] + S[0][3] * S[2][1] * S[3][0]) / det;
    inverse[1][3] =
      (S[0][0] * S[2][1] * S[3][2] - S[0][0] * S[2][2] * S[3][1] - S[0][1] * S[2][0] * S[3][2] + S[0][1] * S[2][2] * S[3][0] + S[0][2] * S[2][0] * S[3][1] - S[0][2] * S[2][1] * S[3][0]) / det;
    inverse[2][0] =
      (S[0][1] * S[1][2] * S[3][3] - S[0][1] * S[1][3] * S[3][2] - S[0][2] * S[1][1] * S[3][3] + S[0][2] * S[1][3] * S[3][1] + S[0][3] * S[1][1] * S[3][2] - S[0][3] * S[1][2] * S[3][1]) / det;
    inverse[2][1] =
      (S[0][0] * S[1][3] * S[3][2] - S[0][0] * S[1][2] * S[3][3] + S[0][2] * S[1][0] * S[3][3] - S[0][2] * S[1][3] * S[3][0] - S[0][3] * S[1][0] * S[3][2] + S[0][3] * S[1][2] * S[3][0]) / det;
    inverse[2][2] =
      (S[0][0] * S[1][1] * S[3][3] - S[0][0] * S[1][3] * S[3][1] - S[0][1] * S[1][0] * S[3][3] + S[0][1] * S[1][3] * S[3][0] + S[0][3] * S[1][0] * S[3][1] - S[0][3] * S[1][1] * S[3][0]) / det;
    inverse[2][3] =
      (S[0][0] * S[1][2] * S[3][1] - S[0][0] * S[1][1] * S[3][2] + S[0][1] * S[1][0] * S[3][2] - S[0][1] * S[1][2] * S[3][0] - S[0][2] * S[1][0] * S[3][1] + S[0][2] * S[1][1] * S[3][0]) / det;
    inverse[3][0] =
      (S[0][1] * S[1][3] * S[2][2] - S[0][1] * S[1][2] * S[2][3] + S[0][2] * S[1][1] * S[2][3] - S[0][2] * S[1][3] * S[2][1] - S[0][3] * S[1][1] * S[2][2] + S[0][3] * S[1][2] * S[2][1]) / det;
    inverse[3][1] =
      (S[0][0] * S[1][2] * S[2][3] - S[0][0] * S[1][3] * S[2][2] - S[0][2] * S[1][0] * S[2][3] + S[0][2] * S[1][3] * S[2][0] + S[0][3] * S[1][0] * S[2][2] - S[0][3] * S[1][2] * S[2][0]) / det;
    inverse[3][2] =
      (S[0][0] * S[1][3] * S[2][1] - S[0][0] * S[1][1] * S[2][3] + S[0][1] * S[1][0] * S[2][3] - S[0][1] * S[1][3] * S[2][0] - S[0][3] * S[1][0] * S[2][1] + S[0][3] * S[1][1] * S[2][0]) / det;
    inverse[3][3] =
      (S[0][0] * S[1][1] * S[2][2] - S[0][0] * S[1][2] * S[2][1] - S[0][1] * S[1][0] * S[2][2] + S[0][1] * S[1][2] * S[2][0] + S[0][2] * S[1][0] * S[2][1] - S[0][2] * S[1][1] * S[2][0]) / det;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            Kg[i][j] = 0;
            for (k = 0; k < 4; k++)
                Kg[i][j] += temp3[i][k] * inverse[k][j];
        }
    }

    //公式3
    for (i = 0; i < 4; i++) {
        temp4[i] = 0;
        for (j = 0; j < 3; j++)
            temp4[i] += H[i][j] * pre[j];
    }

    temp4[0] = ax - temp4[0];
    temp4[1] = ay - temp4[1];
    temp4[2] = az - temp4[2];
    temp4[3] = 1 - temp4[3];

    for (i = 0; i < 3; i++) {
        est[i] = 0;
        for (j = 0; j < 4; j++)
            est[i] += Kg[i][j] * temp4[j];
    }
    for (i = 0; i < 3; i++)
        est[i] += pre[i];

    //公式5
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            temp1[i][j] = 0;
            for (k = 0; k < 4; k++)
                temp1[i][j] += Kg[i][k] * H[k][j];
            temp1[i][j] = -temp1[i][j];
            if (i == j)
                temp1[i][j] += 1;
        }
    }

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            est_p[i][j] = 0;
            for (k = 0; k < 3; k++)
                est_p[i][j] += temp1[i][k] * pre_p[k][j];
        }
    }

    //欧拉角

    if (fabs(est[0]) > 1) {
        if (est[0] > 0) {
            est[0] = 1;
        } else {
            est[0] = -1;
        }
    }

    //    cout<<est[0]<<" "<<est[1]<<" "<<est[2]<<endl;
    ddouble_t standard_one = sqrt(est[0] * est[0] + est[1] * est[1] + est[2] * est[2]);
    // cout<<standard_one<<endl<<endl;

    eular[1] = asin(-est[0] / standard_one);

    eular[0] = asin(est[1] / standard_one / cos(eular[1]));

    for (i = 0; i < 3; i++) {
        est_last[i] = est[i];
    }
}
