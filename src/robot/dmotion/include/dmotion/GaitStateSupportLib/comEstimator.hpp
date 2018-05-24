#pragma once
#include "kalman_filter.hpp"

#include "dcommon/dcommon.hpp"
class comEstimator
{
  public:
    comEstimator();
    void estimateComY(ddouble_t bodyAngleY, int* jointAngle);

  public:
    KalmanFilter* m_filter;

    ddouble_t angle_thres;
    ddouble_t comX;
    ddouble_t comY;
};
