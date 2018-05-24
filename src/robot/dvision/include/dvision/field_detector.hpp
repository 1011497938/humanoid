#pragma once
#include "dmsgs/VisionInfo.h"
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include "dvision/timer.hpp"
#include <ros/ros.h>
#include <vector>

using dmsgs::VisionInfo;
namespace dvision {
class FieldDetector : public IDetector
{
  public:
    explicit FieldDetector();
    ~FieldDetector();
    bool Init();
    void Detect(cv::Mat& hsv_img, cv::Mat& gui_img, Projection& projection, VisionInfo& vision_info, double in_pitch);
    bool Process(cv::Mat& hsv_img, cv::Mat& gui_img, Projection& projection, double in_pitch);

    bool GetPoints(std::vector<cv::Point>& res_points, std::vector<std::vector<cv::Point>>& all_field_contours);
    void FindInField(const cv::Mat& src_hsv_img, const cv::Mat& template_gray_img, cv::Mat* dst_gray_imgs, HSVRange* ranges, bool* in_template, int size = 1);
    std::vector<cv::Point> GetBodyMaskContourInRaw(float rot);

    inline std::vector<cv::Point>& hull_field()
    {
        return hull_field_;
    }

    inline cv::Mat& field_binary_raw()
    {
        return field_binary_raw_;
    }

    inline cv::Mat& field_convex_hull()
    {
        return field_convex_hull_;
    }

    inline cv::Point2f& field_hull_center()
    {
        return field_hull_center_;
    }

    inline cv::Point2f& field_hull_real_center()
    {
        return field_hull_real_center_;
    }

    inline std::vector<cv::Point2f>& field_hull_real()
    {
        return field_hull_real_;
    }

  private:
    std::vector<cv::Point> body_mask_contour_inverted_;
    std::vector<cv::Point> hull_field_;
    std::vector<cv::Point2f> field_hull_real_;
    cv::Point2f field_hull_center_;
    cv::Point2f field_hull_real_center_;
    cv::Mat field_binary_, field_binary_raw_;
    cv::Mat field_convex_hull_;
    int maxDownPixel = 300;
};
} // namespace dvision
