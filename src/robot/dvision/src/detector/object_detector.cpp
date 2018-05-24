/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-28T12:59:03+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: object_detector.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-28T12:59:04+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/object_detector.hpp"

namespace dvision {
ObjectDetector::ObjectDetector()
  : net_(NULL)
  , raw_img_(448, 448, 3, false)
  , ball_position_(0, 0.0, 0, 0, 0, 0)
  , goal_position_(0, 0.0, 0, 0, 0, 0)
  , unknown_position_(0, 0.0, 0, 0, 0, 0)
{
}

ObjectDetector::~ObjectDetector()
{
    delete net_;
}

bool
ObjectDetector::Init()
{
    if (!parameters.object.enable)
        return false;
    // parse label list
    std::string label_file_path = parameters.object.home_folder + "/" + parameters.object.label_file;
    if (!std::ifstream(label_file_path)) {
        ROS_ERROR("darknet label file doesn't exist in %s!", label_file_path.c_str());
        return false;
    }
    label_list_ = darknet::get_labels(label_file_path);

    // parse net cfg and setup network
    std::string net_cfg_path = parameters.object.home_folder + "/" + parameters.object.net_cfg;

    if (!std::ifstream(net_cfg_path)) {
        ROS_ERROR("darknet network cfg doesn't exist in %s!", net_cfg_path.c_str());
        return false;
    }

    // setup network
    net_ = &(darknet::parse_network_cfg(net_cfg_path));
    darknet::params p = net_->get_params();
    ROS_INFO("network setup: num_layers = %d, batch = %d\n", net_->num_layers(), p.batch);

    // load pretrained weights
    std::string weight_file_path = parameters.object.home_folder + "/" + parameters.object.weight_file;
    if (!std::ifstream(weight_file_path)) {
        ROS_ERROR("darknet weigth file doesn't exist in %s!", weight_file_path.c_str());
        return false;
    }
    darknet::load_weights(net_, weight_file_path);

    // set batch to 1 for network inference
    net_->set_network_batch(1);

    ROS_DEBUG("ObjectDetector Init");
    return true;
}

void
ObjectDetector::Detect(const cv::Mat& frame, cv::Mat& gui_img)
{
    Timer t;
    Process(frame, gui_img);
    ROS_DEBUG("object detect used %lf ms", t.elapsedMsec());
}

bool
ObjectDetector::Process(const cv::Mat& frame, cv::Mat& gui_img)
{
    if (!parameters.object.enable) {
        return false;
    }

    // ROS_DEBUG("BallDetector Tick");
    // reinit container
    ReInit();

    // prepare input
    darknet::params p = net_->get_params();
    cv::Mat frame_resized;
    cv::Size size(p.w, p.h);

    cv::resize(frame, frame_resized, size);
    raw_img_.from_mat(frame_resized);

    // get detection result
    darknet::obj_detection(net_, &raw_img_, parameters.object.low_thresh, object_position_relative_);

    // convert relative coord into image coord
    CvtRelativePosition();

    // detection classification
    float ball_max_prob = 0.0;
    float goal_max_prob = 0.0;
    float obstacle_max_prob = 0.0;
    for (auto bbox : object_position_) {
        ROS_DEBUG("find %5d - %5f - (%d, %d) && (%d, %d)", bbox.m_label, bbox.m_prob, bbox.m_left, bbox.m_top, bbox.m_right, bbox.m_bottom);
        if (bbox.m_label == 0 && bbox.m_prob > ball_max_prob && CheckValidScale(bbox.m_left, bbox.m_right, bbox.m_top, bbox.m_bottom, parameters.object.ball_max_scale_coff
            && CheckValidRatio(bbox.m_left, bbox.m_right, bbox.m_top, bbox.m_bottom, parameters.object.ball_wh_low_ratio, parameters.object.ball_wh_high_ratio))) {
            ball_detected_ = true;
            ball_max_prob = bbox.m_prob;
            ball_position_ = bbox;
        } else if (bbox.m_label == 1 && bbox.m_prob > goal_max_prob) {
            goal_detected_ = true;
            goal_max_prob = bbox.m_prob;
            goal_position_ = bbox;
        } else if (bbox.m_label == 2 && bbox.m_prob > obstacle_max_prob && std::abs(bbox.m_left - bbox.m_right) < parameters.camera.width * 0.5 &&
                   std::abs(bbox.m_top - bbox.m_bottom) < parameters.camera.height * 0.5) {
            obstacle_detected_ = true;
            obstacle_max_prob = bbox.m_prob;
            obstacle_positions_.push_back(bbox);
        }
    }

    // show detection results
    if (parameters.monitor.update_gui_img) {
        if (parameters.object.showAllDetections && !object_position_.empty()) {
            for (auto bbox : object_position_) {
                cv::rectangle(gui_img, cv::Point(bbox.m_left, bbox.m_top), cv::Point(bbox.m_right, bbox.m_bottom), darkOrangeColor(), 2);
                cv::putText(gui_img, std::to_string(bbox.m_prob), cv::Point(bbox.m_right, bbox.m_bottom), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, CV_AA);
            }
        }
        if (parameters.object.showBall && ball_detected_) {
            cv::rectangle(gui_img, cv::Point(ball_position_.m_left, ball_position_.m_top), cv::Point(ball_position_.m_right, ball_position_.m_bottom), yellowColor(), 2);
            //            ROS_INFO("ball prob: %f", ball_position_.m_prob);
        }
        if (parameters.object.showGoal && goal_detected_) {
            cv::rectangle(gui_img, cv::Point(goal_position_.m_left, goal_position_.m_top), cv::Point(goal_position_.m_right, goal_position_.m_bottom), redMeloColor(), 2);
        }
        if (parameters.object.showObstacle && obstacle_detected_) {
            for (auto bbox : obstacle_positions_) {
                cv::rectangle(gui_img, cv::Point(bbox.m_left, bbox.m_top), cv::Point(bbox.m_right, bbox.m_bottom), pinkColor(), 2);
            }
        }
    }

    // return if any object is detected
    return object_position_.empty();
}

bool
ObjectDetector::CvtRelativePosition()
{
    for (auto rbbox : object_position_relative_) {
        int left = (rbbox.m_x - rbbox.m_w / 2.) * parameters.camera.width;
        int right = (rbbox.m_x + rbbox.m_w / 2.) * parameters.camera.width;
        int top = (rbbox.m_y - rbbox.m_h / 2.) * parameters.camera.height;
        int bottom = (rbbox.m_y + rbbox.m_h / 2.) * parameters.camera.height;

        if (left < 0) {
            left = 0;
        }
        if (right > parameters.camera.width - 1) {
            right = parameters.camera.width - 1;
        }
        if (top < 0) {
            top = 0;
        }
        if (bottom > parameters.camera.height - 1) {
            bottom = parameters.camera.height - 1;
        }
        object_position_.emplace_back(rbbox.m_label, rbbox.m_prob, left, top, right, bottom);
    }
    return true;
}

bool
ObjectDetector::ReInit()
{
    object_position_.clear();
    object_position_relative_.clear();
    obstacle_positions_.clear();
    ball_detected_ = false;
    goal_detected_ = false;
    obstacle_detected_ = false;

    return true;
}

bool
ObjectDetector::CheckValidScale(const int& left, const int& right, const int& top, const int& bottom, const float& scale_coff)
{
    return (std::abs(left - right) < parameters.camera.width * scale_coff && std::abs(top - bottom) < parameters.camera.height * scale_coff);
}

bool
ObjectDetector::CheckValidRatio(const int& left, const int& right, const int& top, const int& bottom, const float& low_ratio, const float& high_ratio)
{
    int width = std::abs(right - left);
    int height = std::abs(bottom - top);
    if (width != 0 && height != 0) {
        float ratio = static_cast<float>(width) / static_cast<float>(height);
        return (ratio > low_ratio) && (ratio < high_ratio);
    } else {
        return false;
    }
}

} // namespace dvision
