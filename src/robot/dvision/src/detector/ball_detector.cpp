/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-10T10:43:26+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: ball_detector.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-10T10:43:43+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/ball_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
BallDetector::BallDetector()
  : ball_image_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , ball_image_top_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , ball_image_bottom_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , ball_image_radius_(3.0)
  , ball_field_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , ball_field_kalman_(UNKNOWN_OBJ_POS, UNKNOWN_OBJ_POS)
  , kalmanI_(ball_field_kalman_)
{
}

bool
BallDetector::Init()
{
    if (!parameters.ball.enable)
        return false;

    if (parameters.ball.useSimpleBlobDetecor) {
        // setup SimpleBlobDetector
        // Change thresholds
        blob_params_.thresholdStep = parameters.ball.thresholdStep;
        blob_params_.minThreshold = parameters.ball.minThreshold;
        blob_params_.maxThreshold = parameters.ball.maxThreshold;
        blob_params_.minRepeatability = parameters.ball.minRepeatability;
        // Filter by Color.
        blob_params_.filterByColor = parameters.ball.filterByColor;
        blob_params_.blobColor = parameters.ball.blobColor;
        // Filter by Area.
        blob_params_.filterByArea = parameters.ball.filterByArea;
        blob_params_.minArea = parameters.ball.minArea;
        blob_params_.maxArea = parameters.ball.maxArea;
        // Filter by Circularity
        blob_params_.filterByCircularity = parameters.ball.filterByCircularity;
        blob_params_.minCircularity = parameters.ball.minCircularity;
        // Filter by Convexity
        blob_params_.filterByConvexity = parameters.ball.filterByConvexity;
        blob_params_.minConvexity = parameters.ball.minConvexity;
        // Filter by Inertia
        blob_params_.filterByInertia = parameters.ball.filterByInertia;
        blob_params_.minInertiaRatio = parameters.ball.minInertiaRatio;
        // Create SimpleBlobDetector instance
        blob_detector_ = cv::SimpleBlobDetector::create(blob_params_);
    }

    // kalmanI_.Init(ball_field_kalman_);

    ROS_DEBUG("BallDetector Init");
    return true;
}

BallDetector::~BallDetector()
{
}

bool
BallDetector::Update()
{
    if (parameters.ball.useKalman) {
        bool see_ball = kalmanI_.Update(cv::Point2d(ball_field_.x, ball_field_.y));
        ball_field_kalman_ = kalmanI_.GetResult();
        return see_ball;
    }
    return true;
}

void
BallDetector::Detect(const darknet::bbox& ball_position, cv::Mat& gui_img, std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary, Projection& projection, VisionInfo& vision_info)
{
    Timer t;
    vision_info.see_ball = Process(ball_position, gui_img, field_hull_real, field_binary, projection);
    if (vision_info.see_ball) {
        // get ball field
        vision_info.ball_field.x = ball_field().x;
        vision_info.ball_field.y = ball_field().y;
        // get ball velocity
        ball_velocity_ = kalmanI_.GetVelocity();
        vision_info.ball_velocity.x = ball_velocity_.x;
        vision_info.ball_velocity.y = ball_velocity_.y;

        // ROS_INFO("ball_field_raw (%f, %f)", ball_field_.x, ball_field_.y);
        // ROS_INFO("ball_field_kalman (%f, %f)", ball_field_kalman_.x, ball_field_kalman_.y);

        //        std::cerr << ball_field_.x << " " << ball_field_.y << " " << ball_field_kalman_.x << " " << ball_field_kalman_.y << std::endl;
    }
//    ROS_DEBUG("ball detect used %lf ms", t.elapsedMsec());
}

bool
BallDetector::Process(const darknet::bbox& ball_position, cv::Mat& gui_img, std::vector<cv::Point2f>& field_hull_real, cv::Mat& field_binary, Projection& projection)
{
    if (!parameters.ball.enable) {
        return false;
    }

    if (ball_position.m_prob != 0.0) {
        ball_image_.x = (ball_position.m_left + ball_position.m_right) / 2.0;
        ball_image_.y = (ball_position.m_top + ball_position.m_bottom) / 2.0;
        ball_image_top_ = cv::Point(ball_position.m_left, ball_position.m_top);
        ball_image_bottom_ = cv::Point(ball_position.m_right, ball_position.m_bottom);
        // get ball center
        FindBallCenter(field_binary, ball_image_top_, ball_image_bottom_, parameters.ball.BBoxScale, projection);
        projection.getOnRealCoordinate(ball_image_, ball_field_, parameters.field_model.ball_diameter / 2.0);
        CheckBallDist();
        CheckBallRadius(projection);
    } else {
        // ball_image_.x = parameters.kalman.UNKNOWN_POS;
        // ball_image_.y = parameters.kalman.UNKNOWN_POS;
        // ball_field_.x = static_cast<float>(parameters.kalman.UNKNOWN_POS);
        // ball_field_.y = static_cast<float>(parameters.kalman.UNKNOWN_POS);
        // ball_image_.x = NAN_POS;
        // ball_image_.y = NAN_POS;
        ball_field_.x = UNKNOWN_OBJ_POS;
        ball_field_.y = UNKNOWN_OBJ_POS;
    }

    bool see_ball = Update();

    if (parameters.ball.useInFieldCheck) {
        see_ball &= CheckBallInField(field_hull_real);
    }

    if (parameters.monitor.update_gui_img && parameters.ball.showResult && see_ball) {
        // ROS_INFO("ball(%d,%d) radius(%d)", ball_image_.x, ball_image_.y, static_cast<int>(ball_image_radius_));
        cv::circle(gui_img, ball_image_, static_cast<int>(ball_image_radius_), redColor(), 2);
    }

    return see_ball;
}

bool
BallDetector::FindBallCenter(const cv::Mat& field_binary, const cv::Point& ball_image_top, const cv::Point& ball_image_bottom, const float& scale, Projection& projection)
{
    if (!parameters.ball.useSimpleBlobDetecor) {
        return true;
    }

    // enlarge bbox with scale
    float box_width = (ball_image_bottom.x - ball_image_top.x) * scale;
    float box_height = (ball_image_bottom.y - ball_image_top.y) * scale;
    cv::Point bbox_top(ball_image_top.x - box_width * (scale - 1.0) / 2.0, ball_image_top.y - box_height * (scale - 1.0) / 2.0);
    cv::Point bbox_bottom(bbox_top.x + box_width * scale, bbox_top.y + box_height * scale);
    boundry_n(bbox_top.x, 0, parameters.camera.width - 1);
    boundry_n(bbox_top.y, 0, parameters.camera.height - 1);
    boundry_n(bbox_bottom.x, 0, parameters.camera.width - 1);
    boundry_n(bbox_bottom.y, 0, parameters.camera.height - 1);

    // get clipped frame
    if (field_binary.cols != parameters.camera.width && field_binary.rows != parameters.camera.height) {
        return false;
    }
    cv::Rect bbox_init(bbox_top, bbox_bottom);
    cv::Mat frame_clipped = field_binary(bbox_init);

    // detect circular blobs
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(frame_clipped, keypoints);

    cv::Point res_ball;
    cv::Point2f ball_center_real, ball_boundary_real;
    int ball_size = 0;
    float ball_radius_real;
    ball_image_radius_ = 3.0;
    for (auto ball_candidate : keypoints) {
        projection.getOnRealCoordinate(cv::Point(ball_candidate.pt.x + bbox_top.x, ball_candidate.pt.y + bbox_top.y), ball_center_real, parameters.field_model.ball_diameter / 2.0);
        projection.getOnRealCoordinate(
          cv::Point(ball_candidate.pt.x + bbox_top.x + ball_candidate.size / 2.0, ball_candidate.pt.y + bbox_top.y), ball_boundary_real, parameters.field_model.ball_diameter / 2.0);
        ball_radius_real = GetDistance(ball_center_real, ball_boundary_real);
        if (ball_candidate.size > ball_size && ball_radius_real >= parameters.field_model.ball_diameter / 2.0 * parameters.ball.minBallRadiusRatio &&
            ball_radius_real <= parameters.field_model.ball_diameter / 2.0 * parameters.ball.maxBallRadiusRatio) {
            // ROS_INFO("ball radius real: %f", GetDistance(ball_center_real, ball_boundary_real));
            res_ball.x = ball_candidate.pt.x + bbox_top.x;
            res_ball.y = ball_candidate.pt.y + bbox_top.y;
            ball_size = ball_candidate.size;
        }
    }

    if (ball_size > 0 && GetDistance(res_ball, ball_image_) < box_width * parameters.ball.maxBoxCenterOffset) {
        // ROS_INFO("ball center offset %f < %f", GetDistance(res_ball, ball_image_), box_width * parameters.ball.maxBoxCenterOffset);
        ball_image_ = res_ball;
        ball_image_radius_ = ball_size / 2.0;
        return true;
    } else {
        return false;
    }
}

bool
BallDetector::CheckBallInField(std::vector<cv::Point2f>& field_hull_real)
{
    if (!parameters.ball.useInFieldCheck) {
        return true;
    }

    if (field_hull_real.size() > 0) {
        double distance_to_field = cv::pointPolygonTest(field_hull_real, ball_field(), true);
        //        if (distance_to_field < parameters.ball.minBallToFieldDist) {
        //            ROS_INFO("ball (%f, %f) not in field (distance=%f)", ball_field().x, ball_field().y, distance_to_field);
        //        }
        return distance_to_field >= parameters.ball.minBallToFieldDist;
    } else {
        return true;
    }
}

bool
BallDetector::CheckBallDist()
{
    if (!parameters.ball.useDistCheck) {
        return true;
    }

    if (std::isnan(ball_field_.x) && std::isnan(ball_field_.y)) {
        return false;
    }

    if (GetDistance(ball_field_) <= parameters.ball.maxSeeBallDist) {
        return true;
    } else {
        ball_field_.x = UNKNOWN_OBJ_POS;
        ball_field_.y = UNKNOWN_OBJ_POS;
        return false;
    }
}

bool
BallDetector::CheckBallRadius(Projection& projection)
{
    if (!parameters.ball.useRadiusCheck) {
        return true;
    }

    if (std::isnan(ball_field_.x) && std::isnan(ball_field_.y)) {
        return false;
    }

    int ball_width = ball_image_bottom_.x - ball_image_top_.x;
    // int ball_height = ball_image_bottom.y - ball_image_top.y;

    cv::Point2f ball_boundary_real;
    projection.getOnRealCoordinate(cv::Point(ball_image_.x + ball_width / 2.0, ball_image_.y), ball_boundary_real, parameters.field_model.ball_diameter / 2.0);

    float ball_radius_real = GetDistance(ball_field_, ball_boundary_real);
    // ROS_INFO("ball radius real: %f ~ (%f, %f)",
    //          ball_radius_real,
    //          parameters.field_model.ball_diameter / 2.0 * parameters.ball.minBallRadiusRatio,
    //          parameters.field_model.ball_diameter / 2.0 * parameters.ball.maxBallRadiusRatio);

    if (ball_radius_real >= parameters.field_model.ball_diameter / 2.0 * parameters.ball.minBallRadiusRatio &&
        ball_radius_real <= parameters.field_model.ball_diameter / 2.0 * parameters.ball.maxBallRadiusRatio) {
        ball_field_.x = UNKNOWN_OBJ_POS;
        ball_field_.y = UNKNOWN_OBJ_POS;
        return false;
    }
    return true;
}
} // namespace dvision
