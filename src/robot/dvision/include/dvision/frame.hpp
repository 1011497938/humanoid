// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <ros/ros.h>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include "ros_h264_streamer/h264_encoder.h"
#include "ros_h264_streamer/h264_decoder.h"
#include "dvision/parameters.hpp"
#include "dvision/timer.hpp"

namespace dvision {
class Frame
{
  public:
    inline explicit Frame(uint8_t* yuv, int width, int height)
      : m_yuv(yuv)
      , m_width(width)
      , m_height(height)
      , m_converted(false)
    {
        m_timeStamp = ros::Time::now();
    }

    inline Frame(std::string filepath)
    {
        m_bgr = cv::imread(filepath);
        m_width = m_bgr.size().width;
        m_height = m_bgr.size().height;
        m_converted = true;
    }

    inline Frame()
    {
    }

    inline explicit Frame(cv::Mat& mat, int width, int height)
      : m_bgr(mat)
      , m_width(width)
      , m_height(height)
      , m_converted(true)
    {
        m_timeStamp = ros::Time::now();
    }

    inline ~Frame()
    {
    }

    inline cv::Mat getRGB()
    {
        cvt();
        cv::Mat rgb;
        cv::cvtColor(m_bgr, rgb, CV_BGR2RGB);
        return rgb;
    }

    inline cv::Mat& getBGR()
    {
        cvt();
        return m_bgr;
    }

    inline cv::Mat getBGR_raw()
    {
        cvt();
        return m_bgr;
    }

    inline cv::Mat& getHSV()
    {
        cvt();
        return m_hsv;
    }

    inline void cvt()
    {
        if (m_converted)
            return;
        Timer t;
        cv::Mat yuvMat(m_height, m_width, CV_8UC2, m_yuv);

        cv::cvtColor(yuvMat, m_bgr, CV_YUV2BGR_YUYV);
        ROS_DEBUG("yuv to bgr used: %lf ms", t.elapsedMsec());

        cv::cvtColor(m_bgr, m_hsv, CV_BGR2HSV);
        m_converted = true;
        ROS_DEBUG("bgr to hsv used: %lf ms", t.elapsedMsec());

    }

    inline void cudaCvt() {
    }

    void show();

    void save(std::string path);

    // h264 encode & decode
    // Mat --> cv_bridge::CvImage --> ImageMsg --> Encoding --> H264EncoderResult --> raw buffer(pointer, size) --> Decoding --> ImageMsg --> cv_bridge::CvImagePtr --> Mat
    static void initEncoder();

    std::unique_ptr<uint8_t> encode(int& length);

    static std::unique_ptr<uint8_t> encode(const cv::Mat& src, int& length);

    void decode(void* buffer);

  private:
    uint8_t* m_yuv; // raw yuv image
    cv::Mat m_bgr;
    cv::Mat m_hsv;

    ros::Time m_timeStamp;
    int m_width;
    int m_height;
    bool m_converted;

    // h264 encoding
    static ros_h264_streamer::H264Decoder* m_decoder;
    static ros_h264_streamer::H264Encoder* m_encoder;
};
} // namespace dvision
