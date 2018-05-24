// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"

namespace dvision {
DVision::DVision()
{
    m_nh = new ros::NodeHandle("~");
    parameters.init(m_nh);
    m_projection.init(m_nh);
    m_projection2.init(m_nh);
    if(!parameters.simulation) {
        m_obj.Init();
    }
    m_obstacle.Init();
    m_ball.Init();
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_line_classifier.Init();
    m_amcl.Init();
    guiImageBuffer_.Init(4, 30);
    infoBuffer_.Init(3, 30);

    ROS_INFO("%lf %lf", parameters.camera.centerInUndistX, parameters.camera.centerInUndistY);
    m_tracker.Init(parameters.camera.extrinsic_para,
                   parameters.camera.fx,
                   parameters.camera.fy,
                   parameters.camera.undistCx,
                   parameters.camera.undistCy,
                   parameters.camera.centerInUndistX,
                   parameters.camera.centerInUndistY);

    if (!parameters.simulation && (parameters.monitor.update_gui_img || parameters.monitor.transmit_raw_img))
        Frame::initEncoder();

    InitVisionYaw();

    m_sub_motion_info = m_nh->subscribe("/dmotion_" + std::to_string(parameters.robotId) + "/MotionInfo", 1, &DVision::MotionCallback, this);
    m_sub_behaviour_info = m_nh->subscribe("/dbehavior_" + std::to_string(parameters.robotId) + "/BehaviorInfo", 1, &DVision::BehaviourCallback, this);
    m_sub_reload_config = m_nh->subscribe("/humanoid/ReloadVisionConfig", 1, &DVision::ReloadConfigCallback, this);
    m_pub = m_nh->advertise<VisionInfo>("VisionInfo", 1);
    m_toggleAMCL_server = m_nh->advertiseService("toggle_amcl", &DVision::toggle_amcl, this);
    m_resetParticleLeftTouch_server = m_nh->advertiseService("reset_particles_left_touch", &DVision::reset_particles_left_touch, this);
    m_resetParticleRightTouch_server = m_nh->advertiseService("reset_particles_right_touch", &DVision::reset_particles_right_touch, this);
    m_resetParticlePoint_server = m_nh->advertiseService("reset_particles_point", &DVision::reset_particles_point, this);

    if (parameters.simulation) {
        ROS_INFO("Simulation mode  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>!");
        m_transmitter = new dtransmit::DTransmit("127.0.0.1");
        m_transmitter->addRosRecv<VisionInfo>(dconstant::network::monitorBroadcastAddressBase + parameters.robotId, [&](VisionInfo& msg) {
            m_sim_vision_info = msg;
            m_vision_yaw = msg.simYaw / 180.0 * M_PI;
        });
    } else {
        ROS_INFO("Not simulation mode >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>!");
        CameraSettings s(m_nh);
        m_camera = new Camera(s);
        // TODO(MWX): finish this
        if (parameters.camera.debug) {
            m_camera->enableDebugWindow();
        }
        m_transmitter = new dtransmit::DTransmit(parameters.udpBroadcastAddress);
    }
    m_transmitter->startService();

    // Setup concurrent
    if (!parameters.simulation) {
        concurrent_.push([&]() {
            m_obj.Detect(m_frame.getBGR_raw(), workspace_->guiImg);
            workspace_->ballPosition = m_obj.ball_position();
            workspace_->goalPosition = m_obj.goal_position();
        });

        concurrent_.push([&]() {
            m_field.Detect(workspace_->hsvImg, workspace_->guiImg, m_projection, workspace_->visionInfo, workspace_->pitch);
            m_obstacle.Detect(workspace_->hsvImg, workspace_->obstacleMask, workspace_->guiImg, m_field.field_convex_hull(), m_projection, workspace_->visionInfo);

            workspace_->convexHull = m_field.field_convex_hull().clone();
            workspace_->fieldBinaryRaw = m_field.field_binary_raw().clone();
            workspace_->hullField = m_field.hull_field();
            workspace_->fieldHullReal = m_field.field_hull_real();
        });
    } else {
        concurrent_.push([&]() {
            workspace_->visionInfo = m_sim_vision_info;
            workspace_->visionInfo.behaviorInfo = m_behaviour_info;
        });
    }
}

DVision::~DVision()
{
    delete m_camera;
    delete m_transmitter;
}

void
DVision::Start()
{
    t1_ = std::thread([&]() {
        ros::Rate r(30);
        while (ros::ok()) {
            ros::spinOnce();
            Timer t;
            step1();
            ROS_DEBUG("Step1 used %lf ms", t.elapsedMsec());
            r.sleep();
        }
    });

    t2_ = std::thread([&]() {
        ros::Rate r(30);
        while (ros::ok()) {
            Timer t;
            step2();
            ROS_DEBUG("Step2 used %lf ms", t.elapsedMsec());
            r.sleep();
        }
    });

    if (!parameters.simulation && (parameters.monitor.update_gui_img || parameters.monitor.transmit_raw_img)) {
        t3_ = std::thread([&]() {
            ros::Rate r(30);
            size_t cycle = 0;
            while (ros::ok()) {
                Timer t;
                auto& guiImg = guiImageBuffer_.UserRequest();

                if (motionConnected_ && motionStable_) {
                    if (!(++cycle % 15)) {
                        int len;
                        auto buf = Frame::encode(guiImg, len);
                        m_transmitter->sendRaw(dconstant::network::robotGuiBase + parameters.robotId, buf.get(), len);
                    }
                }
                guiImageBuffer_.UserRelease();
                ROS_DEBUG("Update gui image used %lf ms", t.elapsedMsec());
                r.sleep();
            }
        });
    }
}

void
DVision::Join()
{
    t1_.join();
    t2_.join();
    if (parameters.monitor.update_gui_img || parameters.monitor.transmit_raw_img) {
        t3_.join();
    }
}

void
DVision::step1()
{
    Timer t;
    CheckMotionConnection();
    workspace_ = &infoBuffer_.WorkerRequest();

    if(!parameters.simulation) {
        m_frame = m_camera->capture();
    }

    // may need mutex for motionStable_
    if (motionConnected_ && motionStable_ && m_plat_pitch >= 0) {
        workspace_->pitch = m_plat_pitch;
        workspace_->yaw = m_plat_yaw;
        m_projection.updateExtrinsic(m_plat_pitch, m_plat_yaw);

        if (!parameters.simulation) {
            workspace_->visionInfo = VisionInfo();
            workspace_->guiImg = m_frame.getBGR_raw();
            // cvtColor(workspace_->guiImg, workspace_->grayImg, CV_BGR2GRAY); // -2.5
            workspace_->hsvImg = m_frame.getHSV();
        }
        ROS_DEBUG("Pre-processing used %lf ms", t.elapsedMsec());

        concurrent_.spinOnce();
        concurrent_.join();
        ROS_DEBUG("Concurrent used %lf ms", t.elapsedMsec());
    }


    infoBuffer_.WorkerRelease();
}

void
DVision::step2()
{
    auto& visionSharedInfo = infoBuffer_.UserRequest();
    // std::cout << "motionConnected_: " << (int)motionConnected_ << " motionStable: " << (int)motionStable_ << " pitch: " << visionSharedInfo.pitch << std::endl;

    if (motionConnected_ && motionStable_ && visionSharedInfo.pitch >= 0) {
        m_projection2.updateExtrinsic(visionSharedInfo.pitch, visionSharedInfo.yaw);

        if (!parameters.simulation) {
            m_line.Detect(m_canny_img,
                          visionSharedInfo.hsvImg,
                //   visionSharedInfo.grayImg,
                          visionSharedInfo.guiImg,
                          visionSharedInfo.convexHull,
                          visionSharedInfo.fieldBinaryRaw,
                          visionSharedInfo.fieldHullReal,
                          visionSharedInfo.obstacleMask,
                          m_projection2,
                          visionSharedInfo.visionInfo,
                          visionSharedInfo.pitch);

            m_circle.Detect(m_line.result_lines(), m_projection2, visionSharedInfo.visionInfo);
            m_goal.Detect(visionSharedInfo.goalPosition, m_canny_img, visionSharedInfo.hsvImg, visionSharedInfo.guiImg, visionSharedInfo.hullField, m_projection2, visionSharedInfo.visionInfo);
            m_line_classifier.Process(m_line.result_lines(), m_circle.result_circle(), m_goal.goal_position(), m_projection2, visionSharedInfo.visionInfo, visionSharedInfo.yaw);
            m_ball.Detect(visionSharedInfo.ballPosition, visionSharedInfo.guiImg, visionSharedInfo.fieldHullReal, visionSharedInfo.fieldBinaryRaw, m_projection2, visionSharedInfo.visionInfo);
            ShowDebugImg(visionSharedInfo);
        }

        // Protect vision yaw and delta
        std::unique_lock<std::mutex> lk(visionYawLock_);
        m_projection2.UpdateHeadingOffset(m_vision_yaw);
        auto delta = m_delta;
        auto yaw = m_vision_yaw;
        lk.unlock();

        // AMCL
//        if(falled) {
//            m_amcl.falldownGauss();
//            falled = false;
//       }

        m_measurement = Measurement();
        visionSharedInfo.visionInfo.locFieldWhitePoints.clear();
        geometry_msgs::Vector3 tmp;
        if (parameters.simulation) {
            for_each(visionSharedInfo.visionInfo.simFieldWhitePoints.begin(), visionSharedInfo.visionInfo.simFieldWhitePoints.end(), [&](geometry_msgs::Vector3& p) {
                m_measurement.addWhitePoint(cv::Point2f(p.x, p.y));
                visionSharedInfo.visionInfo.locFieldWhitePoints.push_back(p);
            });
        } else {
            m_measurement.fromLines(m_line.result_lines());
            for_each(m_measurement.whitePoints.begin(), m_measurement.whitePoints.end(), [&](cv::Point2f& p) {
                tmp.x = p.x;
                tmp.y = p.y;
                visionSharedInfo.visionInfo.locFieldWhitePoints.push_back(tmp);
            });
        }

        if (visionSharedInfo.visionInfo.see_circle)
            m_measurement.centerPoints.push_back(cv::Point2f(visionSharedInfo.visionInfo.circle_field.x, visionSharedInfo.visionInfo.circle_field.y));

        for (auto& g : visionSharedInfo.visionInfo.goals_field)
            m_measurement.goalPosts.push_back(cv::Point2f(g.x, g.y));
        // prepare measurement end
        m_amcl.Process(m_measurement, delta, yaw / M_PI * 180.0, visionSharedInfo.visionInfo);

        UpdateViewRange(visionSharedInfo.visionInfo);

        // Calc global position
        cv::Point2f ball_global = getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos, visionSharedInfo.visionInfo.ball_field);
        visionSharedInfo.visionInfo.ball_global.x = ball_global.x;
        visionSharedInfo.visionInfo.ball_global.y = ball_global.y;

        // TrackBall
        if (visionSharedInfo.visionInfo.see_ball) {
            if (m_tracker.Process(visionSharedInfo.visionInfo.ball_field.x, visionSharedInfo.visionInfo.ball_field.y)) {
                visionSharedInfo.visionInfo.ballTrack.pitch = Radian2Degree(m_tracker.out_pitch());
                visionSharedInfo.visionInfo.ballTrack.yaw = Radian2Degree(m_tracker.out_yaw());
            }
        }

        // Track Circle
        auto& r = visionSharedInfo.visionInfo.robot_pos;
        auto robotPos = cv::Point3d(r.x, r.y, r.z);
        // calc goal field
        auto circleField = getOnRobotCoordinate(robotPos, cv::Point2f(0, 0));
        if (m_tracker.Process(circleField.x, circleField.y)) {
            visionSharedInfo.visionInfo.circleTrack.pitch = Radian2Degree(m_tracker.out_pitch());
            visionSharedInfo.visionInfo.circleTrack.yaw = Radian2Degree(m_tracker.out_yaw());
        }

        // Track Goal
        auto redGoalCenterField = getOnRobotCoordinate(robotPos, cv::Point2f(-450, 0));
        auto blueGoalCenterField = getOnRobotCoordinate(robotPos, cv::Point2f(450, 0));

        if (m_tracker.Process(redGoalCenterField.x, redGoalCenterField.y)) {
            visionSharedInfo.visionInfo.redGoalTrack.pitch = Radian2Degree(m_tracker.out_pitch());
            visionSharedInfo.visionInfo.redGoalTrack.yaw = Radian2Degree(m_tracker.out_yaw());
        }

        if (m_tracker.Process(blueGoalCenterField.x, blueGoalCenterField.y)) {
            visionSharedInfo.visionInfo.blueGoalTrack.pitch = Radian2Degree(m_tracker.out_pitch());
            visionSharedInfo.visionInfo.blueGoalTrack.yaw = Radian2Degree(m_tracker.out_yaw());
        }

        m_pub.publish(visionSharedInfo.visionInfo);
        if(!(++cycle2_ % 3)) {
            m_transmitter->sendRos(dconstant::network::robotBroadcastAddressBase + parameters.robotId, visionSharedInfo.visionInfo);
        }
    }

    // Remember to release, otherwise dead lock..
    infoBuffer_.UserRelease();

    if (!parameters.simulation && parameters.monitor.update_gui_img) {
        auto& guiImg = guiImageBuffer_.WorkerRequest();
        guiImg = visionSharedInfo.guiImg.clone();
        guiImageBuffer_.WorkerRelease();
    }
}

// TEST(MWX): #1
// Avoid correct heading in init
void
DVision::InitVisionYaw()
{
    m_plat_pitch = 0;
    m_plat_yaw = 0;
    m_imu_roll = 0;
    m_imu_pitch = 0;
    m_imu_yaw_pre = M_PI / 2;
    m_imu_yaw_cur = M_PI / 2;
    m_imu_yaw_delta = 0;
    m_vision_yaw = M_PI / 2;
    m_vision_x_pre = 0;
    m_vision_y_pre = 0;
    m_vision_x_cur = 0;
    m_vision_y_cur = 0;
    m_vision_x_delta = 0;
    m_vision_y_delta = 0;
}

bool
DVision::CheckMotionConnection()
{
    auto now = ros::Time::now();
    auto elapsedSec = (now - lastMotionRecvTime).toSec();
    motionConnected_ = (elapsedSec < 1);
    return motionConnected_;
}

void
DVision::MotionCallback(const dmsgs::MotionInfo::ConstPtr& motion_msg)
{
    lastMotionRecvTime = ros::Time::now();
    m_motion_info = *motion_msg;

    std::lock_guard<std::mutex> lk(visionYawLock_);

    // Reconnect, then init vision yaw

    if (!motionConnected_) {
        ROS_FATAL("Motion reconnected, init vision_yaw");
        InitVisionYaw();
        m_imu_inited = false;
    }

    motionStable_ = m_motion_info.stable;
    m_plat_pitch = m_motion_info.action.headCmd.pitch;
    m_plat_yaw = m_motion_info.action.headCmd.yaw;
    m_imu_roll = m_motion_info.imuRPY.x;
    m_imu_pitch = m_motion_info.imuRPY.y;

    if (!m_imu_inited) {
        m_imu_yaw_pre = m_motion_info.imuRPY.z;
        m_imu_yaw_cur = m_motion_info.imuRPY.z;
        m_imu_inited = true;
    } else {
        m_imu_yaw_pre = m_imu_yaw_cur;
        m_imu_yaw_cur = m_motion_info.imuRPY.z;
        m_imu_yaw_delta = m_imu_yaw_cur - m_imu_yaw_pre;
        m_vision_yaw += m_imu_yaw_delta;
        //        std::cout << m_imu_yaw_delta << " " << m_vision_yaw << std::endl;
    }

    if (m_vision_yaw > M_PI) {
        m_vision_yaw -= 2 * M_PI;
    } else if (m_vision_yaw <= -M_PI) {
        m_vision_yaw += 2 * M_PI;
    }

    {
        auto tmpDelta = m_motion_info.deltaData;
        auto dx = tmpDelta.x - m_previous_delta.x;
        auto dy = tmpDelta.y - m_previous_delta.y;
        auto dt = tmpDelta.z - m_previous_delta.z;

        auto t = Degree2Radian(m_previous_delta.z);
        auto ddx = dx * cos(-t) - dy * sin(-t);
        auto ddy = dx * sin(-t) + dy * cos(-t);

        m_vision_x_pre = m_vision_x_cur;
        m_vision_y_pre = m_vision_y_cur;

        m_vision_x_cur += ddx * cos(m_vision_yaw) - ddy * sin(m_vision_yaw);
        m_vision_y_cur += ddx * sin(m_vision_yaw) + ddy * cos(m_vision_yaw);

        if (!motionStable_) {
            m_vision_x_delta = 0;
            m_vision_y_delta = 0;
        } else {
            m_vision_x_delta = m_vision_x_cur - m_vision_x_pre;
            m_vision_y_delta = m_vision_y_cur - m_vision_y_pre;
        }

        m_previous_delta = tmpDelta;

        if (parameters.calib.calibOdometer) {
            m_vision_x_sum += m_vision_x_delta;
            m_vision_y_sum += m_vision_y_delta;
            dx_sum_ += dx;
            dy_sum_ += dy;
            dt_sum_ += dt;
            std::cout << "m_vision_x_sum: " << m_vision_x_sum << std::endl;
            std::cout << "m_vision_y_sum: " << m_vision_y_sum << std::endl;
            std::cout << "dx_sum_:" << dx_sum_ << std::endl;
            std::cout << "dy_sum_:" << dy_sum_ << std::endl;
            std::cout << "dt_sum_:" << dt_sum_ << std::endl;
        }

        if (parameters.simulation) {
            m_delta.dx = ddx;
            m_delta.dy = ddy;
            m_delta.dt = dt;
        } else {
            m_delta.dx = m_vision_x_delta;
            m_delta.dy = m_vision_y_delta;
            m_delta.dt = dt;
        }
    }
}

void
DVision::BehaviourCallback(const dmsgs::BehaviorInfo::ConstPtr& behaviour_msg)
{
    m_behaviour_info = *behaviour_msg;
    if (!parameters.simulation && m_behaviour_info.save_image && !saveImg_) {
        saveImg_ = true;
        std::string path_str;
        path_str = "p_" + std::to_string(m_plat_pitch) + "_y_" + std::to_string(m_plat_yaw) + " ";
        ROS_INFO("save_image! %s", path_str.c_str());
        auto frame = m_camera->capture();
        frame.save(path_str);
    }
    saveImg_ = m_behaviour_info.save_image;
}

void
DVision::ReloadConfigCallback(const std_msgs::String::ConstPtr&)
{
    parameters.update();
}

void
DVision::UpdateViewRange(VisionInfo& info)
{
    Timer t;
    cv::Point2f upperLeft;
    cv::Point2f upperRight;
    cv::Point2f lowerLeft;
    cv::Point2f lowerRight;

    m_projection2.getOnRealCoordinate(cv::Point(0, 0), upperLeft);
    m_projection2.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, 0), upperRight);
    m_projection2.getOnRealCoordinate(cv::Point(0, parameters.camera.height - 1), lowerLeft);
    m_projection2.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, parameters.camera.height - 1), lowerRight);

    auto v1 = upperLeft - lowerLeft;
    auto v2 = lowerRight - lowerLeft;
    auto theta = getAngleBetweenVectors(v1, v2);
    if (theta < 180 && theta > 0) {
        upperLeft *= -100;
    }

    v1 = upperRight - lowerRight;
    v2 = -v2;
    theta = getAngleBetweenVectors(v2, v1);
    if (theta < 180 && theta > 0) {
        upperRight *= -100;
    }

    info.viewRange.resize(4);
    info.viewRange[0].x = upperLeft.x;
    info.viewRange[0].y = upperLeft.y;
    info.viewRange[1].x = upperRight.x;
    info.viewRange[1].y = upperRight.y;
    info.viewRange[2].x = lowerRight.x;
    info.viewRange[2].y = lowerRight.y;
    info.viewRange[3].x = lowerLeft.x;
    info.viewRange[3].y = lowerLeft.y;
}

void
DVision::ShowDebugImg(VisionSharedInfo& info)
{
    if (parameters.simulation)
        return;

    Timer t;
    if (parameters.monitor.use_cv_show) {
        if (parameters.monitor.update_canny_img && info.visionInfo.see_line) {
            cv::namedWindow("canny", CV_WINDOW_NORMAL);
            cv::imshow("canny", m_canny_img);
        }

        if (parameters.monitor.update_field_binary) {
            m_field_binary = cv::Mat::zeros(info.hsvImg.size(), CV_8UC1);
            cv::inRange(
              info.hsvImg, cv::Scalar(parameters.field.h0, parameters.field.s0, parameters.field.v0), cv::Scalar(parameters.field.h1, parameters.field.s1, parameters.field.v1), m_field_binary);
            cv::namedWindow("field_binary", CV_WINDOW_NORMAL);
            cv::imshow("field_binary", m_field_binary);
            cv::createTrackbar("h_low", "field_binary", &parameters.field.h0, 255);
            cv::createTrackbar("h_high", "field_binary", &parameters.field.h1, 255);
            cv::createTrackbar("s_low", "field_binary", &parameters.field.s0, 255);
            cv::createTrackbar("s_high", "field_binary", &parameters.field.s1, 255);
            cv::createTrackbar("v_low", "field_binary", &parameters.field.v0, 255);
            cv::createTrackbar("v_high", "field_binary", &parameters.field.v1, 255);
        }

        if (parameters.monitor.update_goal_binary) {
            m_goal_binary = cv::Mat::zeros(info.hsvImg.size(), CV_8UC1);
            cv::inRange(info.hsvImg, cv::Scalar(parameters.goal.h0, parameters.goal.s0, parameters.goal.v0), cv::Scalar(parameters.goal.h1, parameters.goal.s1, parameters.goal.v1), m_goal_binary);
            cv::namedWindow("goal_binary", CV_WINDOW_NORMAL);
            cv::imshow("goal_binary", m_goal_binary);
            cv::createTrackbar("h_low", "goal_binary", &parameters.goal.h0, 255);
            cv::createTrackbar("h_high", "goal_binary", &parameters.goal.h1, 255);
            cv::createTrackbar("s_low", "goal_binary", &parameters.goal.s0, 255);
            cv::createTrackbar("s_high", "goal_binary", &parameters.goal.s1, 255);
            cv::createTrackbar("v_low", "goal_binary", &parameters.goal.v0, 255);
            cv::createTrackbar("v_high", "goal_binary", &parameters.goal.v1, 255);
        }

        if (parameters.monitor.update_line_binary) {
            m_line_binary = cv::Mat::zeros(info.hsvImg.size(), CV_8UC1);
            cv::inRange(info.hsvImg, cv::Scalar(parameters.line.h0, parameters.line.s0, parameters.line.v0), cv::Scalar(parameters.line.h1, parameters.line.s1, parameters.line.v1), m_line_binary);
            cv::namedWindow("line_binary", CV_WINDOW_NORMAL);
            cv::imshow("line_binary", m_line_binary);
            cv::createTrackbar("h_low", "line_binary", &parameters.line.h0, 255);
            cv::createTrackbar("h_high", "line_binary", &parameters.line.h1, 255);
            cv::createTrackbar("s_low", "line_binary", &parameters.line.s0, 255);
            cv::createTrackbar("s_high", "line_binary", &parameters.line.s1, 255);
            cv::createTrackbar("v_low", "line_binary", &parameters.line.v0, 255);
            cv::createTrackbar("v_high", "line_binary", &parameters.line.v1, 255);
        }

        if (parameters.monitor.update_obstacle_binary && info.visionInfo.see_field) {
            cv::namedWindow("obstacle_binary", CV_WINDOW_NORMAL);
            if(info.obstacleMask.cols > 0 && info.obstacleMask.rows > 0){
                cv::imshow("obstacle_binary", info.obstacleMask);

                // using for change hsv of white
                cv::createTrackbar("h_low", "obstacle_binary", &parameters.obstacle.h0, 255);
                cv::createTrackbar("h_high", "obstacle_binary", &parameters.obstacle.h1, 255);
                cv::createTrackbar("s_low", "obstacle_binary", &parameters.obstacle.s0, 255);
                cv::createTrackbar("s_high", "obstacle_binary", &parameters.obstacle.s1, 255);
                cv::createTrackbar("v_low", "obstacle_binary", &parameters.obstacle.v0, 255);
                cv::createTrackbar("v_high", "obstacle_binary", &parameters.obstacle.v1, 255);
            }
        }

        if (parameters.monitor.update_gui_img) {
            cv::namedWindow("gui", CV_WINDOW_NORMAL);
            cv::imshow("gui", info.guiImg);
        }
        cv::waitKey(1);
    }
    ROS_DEBUG("show debug image used: %lf ms", t.elapsedMsec());
}

/*
cv::Mat obstacleMask;
cv::Mat convexHull;
cv::Mat fieldBinaryRaw;
std::vector<cv::Point> hullField;
std::vector<cv::Point2f> fieldHullReal;
std::vector<LineSegment> goodLines;
*/
bool DVision::toggle_amcl(dmsgs::ToggleAMCL::Request &req, dmsgs::ToggleAMCL::Response &res)
{
    enableAMCL_ = (bool)req.swtich;
    ROS_DEBUG("toggle amcl %s", (bool)req.swtich ? "true" : "false");
    return true;
}

bool DVision::reset_particles_left_touch(dmsgs::ResetParticleLeftTouch::Request &req,
                                         dmsgs::ResetParticleLeftTouch::Response &res)
{
    ROS_DEBUG("reset particles left touch");
    m_amcl.ResetParticlesLeftTouch();
    return true;
}

bool DVision::reset_particles_right_touch(dmsgs::ResetParticleRightTouch::Request &req,
                                          dmsgs::ResetParticleRightTouch::Response &res)
{
    ROS_DEBUG("reset particles right touch");
    m_amcl.ResetParticlesRightTouch();
    return true;
}

bool DVision::reset_particles_point(dmsgs::ResetParticlePoint::Request &req,
                                    dmsgs::ResetParticlePoint::Response &res)
{
    auto& point = req.point;
    m_amcl.ResetParticlesPoint(point);
    ROS_DEBUG("reset particles at point %lf %lf %lf", point.x, point.y, point.z);
    return true;
}

} // namespace dvision
