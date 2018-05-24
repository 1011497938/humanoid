/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-05T19:24:53+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: kalman.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-05T19:24:56+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/kalman.hpp"

namespace dvision {
ObjectPosKalmanFilter::ObjectPosKalmanFilter(const cv::Point2f& p)
  : last_est_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , curr_est_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , curr_pred_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , reset_(false)
  , state_size_(4)
  , measure_size_(2)
  , control_size_(0)
{
    Init(p);
}

ObjectPosKalmanFilter::~ObjectPosKalmanFilter()
{
    delete kalman_;
}

void
ObjectPosKalmanFilter::Init(const cv::Point2f& p)
{
    // renew a pointer to KalmanFilter instance
    kalman_ = new cv::KalmanFilter(state_size_, measure_size_, control_size_, CV_32F);
//    kalman_ = new cv::KalmanFilter(4, 2, 0, CV_32F);

    // Predicted State (x'(k))
    // x(k) = A * x(k-1) + B * u(k)
    // [x, y, v_x, v_y]
    kalman_->statePre.at<float>(0) = p.x;
    kalman_->statePre.at<float>(1) = p.y;
    kalman_->statePre.at<float>(2) = 0;
    kalman_->statePre.at<float>(3) = 0;

    // Transition State Matrix A
    // [ 1,  0,  dx, 0  ]
    // [ 0,  1,  0,  dy ]
    // [ 0,  0,  1,  0  ]
    // [ 0,  0,  0,  1  ]
    cv::setIdentity(kalman_->transitionMatrix, cv::Scalar(1));
    // TODO(corenel) set dT to real time interval
    kalman_->transitionMatrix.at<float>(2) = 1.0f;
    kalman_->transitionMatrix.at<float>(7) = 1.0f;
    // kalman_->transitionMatrix = (cv::Mat_<float>(state_size_, state_size_) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

    // Measure Matrix H
    // [ 1, 0, 0, 0 ]
    // [ 0, 1, 0, 0 ]
    kalman_->measurementMatrix = cv::Mat::zeros(measure_size_, state_size_, CV_32F);
    kalman_->measurementMatrix.at<float>(0) = 1.0f;
    kalman_->measurementMatrix.at<float>(5) = 1.0f;

    // TODO(corenel) add control
    // Control Matrix B
    // [ 1, 0 ]
    // [ 0, 1 ]
    // cv::setIdentity(kalman_->controlMatrix, cv::Scalar(1e-4));

    // Process Noise Covariance Matrix Q
    // [ Ex   0    0     0    ]
    // [ 0    Ey   0     0    ]
    // [ 0    0    Edx   0    ]
    // [ 0    0    0     Edy  ]
    cv::setIdentity(kalman_->processNoiseCov, cv::Scalar(1e-4));
    // kalman_->processNoiseCov.at<float>(0) = 1e-2;
    // kalman_->processNoiseCov.at<float>(5) = 1e-2;
    // kalman_->processNoiseCov.at<float>(10) = 5.0f;
    // kalman_->processNoiseCov.at<float>(15) = 5.0f;

    // MeasureNoise Covariance Matrix R
    cv::setIdentity(kalman_->measurementNoiseCov, cv::Scalar::all(1e-1));
    // cv::setIdentity(kalman_->measurementNoiseCov, cv::Scalar::all(25));

    // Posteriori Error Estimate Covariance Matrix P
    cv::setIdentity(kalman_->errorCovPost, cv::Scalar::all(1e-1));
}

bool
ObjectPosKalmanFilter::Update(const cv::Point2f& p)
{
    // ROS_INFO("-----------kalman tick-------------");
    GetPrediction();
    if (std::isnan(p.x) && std::isnan(p.y)) {
        // ROS_WARN("not see ball %d", miss_count_);
        // TODO(corenel) compare last_est_ and curr_pred_
        curr_est_ = last_est_;
        // curr_est_ = curr_pred_;
        if (miss_time_.getElapsedSec() > parameters.kalman.maxMissSec) {
            // ROS_WARN("-----------------Miss Ball!!! %d", miss_count_);
            reset_ = true;
            return false;
        }
    } else {
        // ROS_INFO("see ball");
        miss_time_.restart();
        if (reset_ && parameters.kalman.allowReset) {
            // ROS_WARN("-----------------Reset Kalman Filter %d", miss_count_);
            Init(p);
            last_est_ = p;
            curr_est_ = p;
            reset_ = false;
        } else {
            cv::Mat measurement(measure_size_, 1, CV_32FC1);

            measurement.at<float>(0) = p.x;
            measurement.at<float>(1) = p.y;

            // Corrected State (x(k))
            // x(k) = x'(k) + G(k) * (z(k) - H * x'(k))
            cv::Mat estimated = kalman_->correct(measurement);
            curr_est_ = cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
            last_est_ = curr_est_;
        }
    }
    return true;
}

cv::Point2f
ObjectPosKalmanFilter::GetPrediction()
{
    // update dT in A (decreasing udpate speed of object position, disabled)
    // float dT = diff_time_.elapsedSec();
    // dT = dT > 0.2 ? 0.2 : dT;
    // kalman_->transitionMatrix.at<float>(2) = dT;
    // kalman_->transitionMatrix.at<float>(7) = dT;
    // ROS_INFO("dT: %f", dT);

    // TODO(corenel) add control matrix
    cv::Mat prediction = kalman_->predict();
    curr_pred_ = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
    return curr_pred_;
}

cv::Point2f
ObjectPosKalmanFilter::GetResult()
{
    return curr_est_;
}

cv::Point2f
ObjectPosKalmanFilter::GetVelocity()
{

    // ROS_INFO("ball delta (dx, dy) = (%f, %f)", kalman_->statePre.at<float>(2), kalman_->statePre.at<float>(3));
    float dT = diff_time_.elapsedSec();
    dT = dT > 0.2 ? 0.2 : dT;
    // ROS_INFO("dT: %f", dT);
    // ROS_INFO("ball velocity (vx, vy) = (%f, %f)", kalman_->statePre.at<float>(2) / dT, kalman_->statePre.at<float>(3) / dT);

    // return cv::Point2f(kalman_->statePre.at<float>(2), kalman_->statePre.at<float>(3));
    return cv::Point2f(kalman_->statePre.at<float>(2) / dT, kalman_->statePre.at<float>(3) / dT);
}
} // namespace dvision
