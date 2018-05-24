#include "dmotion/GaitStateSupportLib/comEstimator.hpp"
#include <cmath>
comEstimator::comEstimator()
{
    comX = 0.000;
    comY = 0.000;
    angle_thres = 15.0 / 180 * M_PI;

    m_filter = new KalmanFilter();
}
