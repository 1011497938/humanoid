// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <thread>
#include <mutex>
#include "dmsgs/ActionCommand.h"
#include "dmsgs/BehaviorInfo.h"
#include "dmsgs/MotionInfo.h"
#include "dmsgs/SaveImg.h"
#include "std_msgs/String.h"

#include "dmsgs/VisionInfo.h"

#include "dmsgs/ToggleAMCL.h"
#include "dmsgs/ResetParticleLeftTouch.h"
#include "dmsgs/ResetParticleRightTouch.h"
#include "dmsgs/ResetParticlePoint.h"

#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/amcl/amcl.hpp"
#include "dvision/ball_detector.hpp"
#include "dvision/ball_tracker.hpp"
#include "dvision/camera.hpp"
#include "dvision/circle_detector.hpp"
#include "dvision/field_detector.hpp"
#include "dvision/goal_detector.hpp"
#include "dvision/line_detector.hpp"
#include "dvision/line_classifier.hpp"
#include "dvision/object_detector.hpp"
#include "dvision/obstacle_detector.hpp"
#include "dvision/projection.hpp"
#include "dvision/utils.hpp"
#include "dvision/ringbuffer.hpp"
#include "dvision/vision_shared_info.hpp"

namespace dvision {

class DVision
{
  public:
    explicit DVision();
    ~DVision();
    void Start();
    void Join();

  private:
    // For parallel
    RingBuffer<cv::Mat> guiImageBuffer_;
    RingBuffer<VisionSharedInfo> infoBuffer_;

    void step1();
    void step2();
    std::thread t1_;
    std::thread t2_;
    std::thread t3_;

private:
    ros::Subscriber m_sub_motion_info;
    ros::Subscriber m_sub_behaviour_info;
    ros::Subscriber m_sub_reload_config;
    ros::Publisher m_pub;

    ros::ServiceServer m_toggleAMCL_server;
    ros::ServiceServer m_resetParticleLeftTouch_server;
    ros::ServiceServer m_resetParticleRightTouch_server;
    ros::ServiceServer m_resetParticlePoint_server;

    dtransmit::DTransmit* m_transmitter;
    std::mutex visionYawLock_;

private:

    // Variables for thread1
    ros::NodeHandle* m_nh;
    Camera* m_camera;
    Projection m_projection;
    dprocess::DConcurrent concurrent_;
    VisionSharedInfo* workspace_;
    Frame m_frame;
//    cv::Mat m_hsv_img, m_gray_img;
    cv::Mat m_field_binary, m_goal_binary, m_line_binary, m_obstacle_binary;
//    cv::Mat m_gui_img;
    cv::Mat m_canny_img;
    size_t cycle1_ = 0;

    double m_plat_pitch = 0;
    double m_plat_yaw = 0;

    // Detectors
    FieldDetector m_field;
    ObjectDetector m_obj;
    ObstacleDetector m_obstacle;

private:
    // Variables for thread2
    Tracker m_tracker;
    Projection m_projection2;

    AMCL m_amcl;
    Control m_delta;
    geometry_msgs::Vector3 m_previous_delta;
    Pose m_pose;
    Measurement m_measurement;
    bool enableAMCL_ = true;

    // Detectors
    BallDetector m_ball;
    CircleDetector m_circle;
    LineDetector m_line;
    LineClassifier m_line_classifier;
    GoalDetector m_goal;
    size_t cycle2_ = 0;

    // Vision yaw
    bool m_imu_inited = false;
    double m_imu_roll = 0;
    double m_imu_pitch = 0;
    double m_imu_yaw_pre = M_PI / 2;
    double m_imu_yaw_cur = M_PI / 2;
    double m_imu_yaw_delta = 0;
    double m_vision_yaw = M_PI / 2;         // 机器人yaw角度

    double m_vision_x_pre = 0;   // 机器人前一帧的世界坐标系位置 x
    double m_vision_y_pre = 0;   // 机器人前一帧的世界坐标系位置 y
    double m_vision_x_cur = 0;   // 机器人当前帧的世界坐标系位置 x
    double m_vision_y_cur = 0;   // 机器人当前帧的世界坐标系位置 y
    double m_vision_x_delta = 0; // 两帧间的世界坐标系位置差 delta_x
    double m_vision_y_delta = 0; // 两帧间的世界坐标系位置差 delta_y

    // using for test
    double m_vision_x_sum = 0; // 两帧间的世界坐标系位置差 delta_x
    double m_vision_y_sum = 0; // 两帧间的世界坐标系位置差 delta_y
    double dx_sum_ = 0;
    double dy_sum_ = 0;
    double dt_sum_ = 0;

private:
    bool falled = false;
    bool saveImg_ = false;

    // Handle motion disconnected and fall down
    ros::Time lastMotionRecvTime = ros::Time(0);
    bool CheckMotionConnection();
    bool motionConnected_ = false;
    bool motionStable_ = false;

    // Handle penalty


    //dmsgs::VisionInfo m_vision_info;
    dmsgs::VisionInfo m_sim_vision_info;
    dmsgs::MotionInfo m_motion_info;
    dmsgs::BehaviorInfo m_behaviour_info;

    void InitVisionYaw();
    void ShowDebugImg(VisionSharedInfo& info);
    void TrackBall();
    void UpdateViewRange(VisionInfo& info);

    void MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg);
    void BehaviourCallback(const dmsgs::BehaviorInfo::ConstPtr& msg);
    void ReloadConfigCallback(const std_msgs::String::ConstPtr&);

    // Service
    bool toggle_amcl(dmsgs::ToggleAMCL::Request& req, dmsgs::ToggleAMCL::Response& res);
    bool reset_particles_left_touch(dmsgs::ResetParticleLeftTouch::Request& req, dmsgs::ResetParticleLeftTouch::Response& res);
    bool reset_particles_right_touch(dmsgs::ResetParticleRightTouch::Request& req, dmsgs::ResetParticleRightTouch::Response& res);
    bool reset_particles_point(dmsgs::ResetParticlePoint::Request& req, dmsgs::ResetParticlePoint::Response& res);

};
} // namespace dvision
