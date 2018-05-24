// Copied from Nimbro
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {

class DistortionModel
{
  public:
    DistortionModel();
    void init();

    cv::Point2f undistort(int x, int y);
    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);
    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res);
    void undistortImage(const cv::Mat& src, cv::Mat& dst);
    void undistortImage2(const cv::Mat& src, cv::Mat& dst);
    void undistortImage3(const cv::Mat& src, cv::Mat& dst);

    void distort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);
    cv::Point distort(int x, int y);
    inline cv::Point distort(const cv::Point point)
    {
        return distort(point.x, point.y);
    }
    bool distortP(const cv::Point& point, cv::Point& resPoint);
    bool distortP(const std::vector<cv::Point>& contour, std::vector<cv::Point>& resCountour);

  private:
    bool undistort_slow(const std::vector<cv::Point>& points, std::vector<cv::Point>& resPoints);

  private:
    std::vector<cv::Point> m_undistortionVector; // undist to dist
    std::vector<cv::Point> m_distortionVector;   // dist to undist
    cv::Mat m_map1;
    cv::Mat m_map2;
};
} // namespace dvision
