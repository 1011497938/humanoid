// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <cstdint>
#include <linux/videodev2.h>
#include <ros/ros.h>
#include <string>

namespace dvision {
struct CameraSettings
{
    inline explicit CameraSettings()
      : device("/dev/video0")
      , width(640)
      , height(480)
      , frameRate(30)
      , brightness(128)
      , contrast(128)
      , saturation(128)
      , hue(0)
      , sharpness(0)
      , gain(255)
      , gamma(27)
      , whitebalance_auto(0)
      , whitebalance_absolute(4500)
      , exposure_auto(1)
      , exposure_absolute(150)
      , focus_auto(1)
      , focus_absolute(0)
    {
    }

#define GPARAM(x)                                                                                                                                                                                      \
    do {                                                                                                                                                                                               \
        if (!nh->getParam("dvision/camera/" #x, x)) {                                                                                                                                                 \
            ROS_FATAL("CameraSettings get pararm error");                                                                                                                                              \
            throw std::runtime_error("Camera settings get param error!");                                                                                                                              \
        }                                                                                                                                                                                              \
    } while (0)

    inline explicit CameraSettings(ros::NodeHandle* nh)
    {
        GPARAM(device);
        GPARAM(width);
        GPARAM(height);
        GPARAM(frameRate);
        GPARAM(brightness);
        GPARAM(contrast);
        GPARAM(saturation);
        GPARAM(hue);
        GPARAM(sharpness);
        GPARAM(gain);
        GPARAM(gamma);
        GPARAM(whitebalance_auto);
        GPARAM(whitebalance_absolute);
        GPARAM(exposure_auto);
        GPARAM(exposure_absolute);
        GPARAM(focus_auto);
        GPARAM(focus_absolute);

        ROS_INFO("Camera settings, width: %d height: %d", width, height);
    }
#undef GPARAM

    std::string device;
    int width;
    int height;
    int frameRate;
    int brightness;
    int contrast;
    int saturation;
    int hue;
    int sharpness;
    int gain;
    int gamma;

    int whitebalance_auto;
    int whitebalance_absolute;
    int exposure_auto;
    int exposure_absolute;
    int focus_auto;
    int focus_absolute;
};

enum class V4L2CID
{
    white_balance_auto = V4L2_CID_AUTO_WHITE_BALANCE,
    white_balance_temperature = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
    exposure_auto = V4L2_CID_EXPOSURE_AUTO,
    exposure_absolute = V4L2_CID_EXPOSURE_ABSOLUTE,
    focus_auto = V4L2_CID_FOCUS_AUTO,
    focus_absolute = V4L2_CID_FOCUS_ABSOLUTE,
    brightness = V4L2_CID_BRIGHTNESS,
    contrast = V4L2_CID_CONTRAST,
    saturation = V4L2_CID_SATURATION,
    sharpness = V4L2_CID_SHARPNESS,
    gain = V4L2_CID_GAIN,
    hue = V4L2_CID_HUE,
    gamma = V4L2_CID_GAMMA
};
} // namespace dvision
