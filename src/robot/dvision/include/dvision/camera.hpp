// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dvision/cameraSettings.hpp"
#include "dvision/frame.hpp"
#include <linux/videodev2.h>

namespace dvision {
class Camera
{
  public:
    // use default/saved camera parameters
    explicit Camera(std::string device = "/dev/video0");

    // set parameters explicitly
    explicit Camera(CameraSettings);

    ~Camera();

    Frame capture();

    void setControl(V4L2CID id, const uint32_t value);

    void setControl(const uint32_t cid, const uint32_t value);

    v4l2_queryctrl getControl(V4L2CID cid);

    v4l2_queryctrl getControl(uint32_t cid);

    void enableDebugWindow();

    // prohibit copy/move
    Camera(const Camera&) = delete;
    Camera(Camera&&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera& operator=(Camera&&) = delete;

  private:
    void init();
    void deInit();
    void openDevice();
    void closeDevice();
    void initFmt();
    void setFrameRate(uint32_t, uint32_t);
    void startIO();
    void stopIO();
    void doIO();
    void setCameraControl();
    void resetControl();
    void initMmap();



  private:
    std::string m_device;
    int m_fd;
    CameraSettings m_setting;

    static const int NUM_FRAME_BUFFERS = 3;
    struct
    {
        uint8_t* start;
        size_t length;
    } buffers[NUM_FRAME_BUFFERS];

    uint32_t n_buffers;
    v4l2_buffer lastDequeued;

    // Pointer to last dequeued buffer
    void* raw_yuv;
    uint32_t raw_yuv_size;
    bool showDebugWindow_ = false;

};
} // namespace dvision
