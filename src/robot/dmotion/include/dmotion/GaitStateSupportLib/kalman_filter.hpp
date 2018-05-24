#pragma once

#include "dcommon/dcommon.hpp"
namespace KALMAN {
extern ddouble_t T;
}

class KalmanFilter
{
  public:
    KalmanFilter();
    void estimate(ddouble_t wx, ddouble_t wy, ddouble_t wz, ddouble_t ax, ddouble_t ay, ddouble_t az, ddouble_t time = 0.018); // time is in seconds
    ddouble_t eular[2];

  private:
    ddouble_t A[3][3];
    ddouble_t est_last[3];
    ddouble_t est_p[3][3];
};
