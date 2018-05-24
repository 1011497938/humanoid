#include "dvision/parameters.hpp"
#include <vector>
using std::vector;
using namespace cv;

namespace dvision {

#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!m_nh->getParam(x, y)) {                                                                                                                                                                   \
            ROS_FATAL("Projection get pararm " #x " error!");                                                                                                                                          \
        }                                                                                                                                                                                              \
    } while (0)

void
Parameters::init(ros::NodeHandle* nh)
{
    m_nh = nh;
    update();
}

void
Parameters::update()
{
    // Global parameters
    GPARAM("/ZJUDancer/Simulation", parameters.simulation);
    GPARAM("/ZJUDancer/udpBroadcastAddress", parameters.udpBroadcastAddress);

    // Private parameters
    GPARAM("RobotId", parameters.robotId);

    //    GPARAM("dvision/camera/debug", parameters.camera.debug);
    GPARAM("dvision/projection/fx", parameters.camera.fx);
    GPARAM("dvision/projection/fy", parameters.camera.fy);
    GPARAM("dvision/projection/cx", parameters.camera.cx);
    GPARAM("dvision/projection/cy", parameters.camera.cy);
    GPARAM("dvision/projection/extrinsic_para", parameters.camera.extrinsic_para);
    GPARAM("dvision/camera/width", parameters.camera.width);
    GPARAM("dvision/camera/height", parameters.camera.height);

    parameters.camera.cameraMatrix = (Mat_<double>(3, 3) << parameters.camera.fx, 0, parameters.camera.cx, 0, parameters.camera.fy, parameters.camera.cy, 0, 0, 1);
    parameters.camera.distCoeff = Mat_<double>(1, 14);

    vector<double> dist_coeff;
    GPARAM("dvision/projection/dist_coeff", dist_coeff);
    for (uint32_t i = 0; i < dist_coeff.size(); ++i) {
        parameters.camera.distCoeff.at<double>(0, i) = dist_coeff[i];
    }

    parameters.camera.imageSize = Size(parameters.camera.width, parameters.camera.height);

    // Calib
    GPARAM("dvision/calib/calibOdometer", parameters.calib.calibOdometer);

    // Get kalman filter parameters
    GPARAM("dvision/kalman/maxMissSec", parameters.kalman.maxMissSec);
    GPARAM("dvision/kalman/allowReset", parameters.kalman.allowReset);

    // GPARAM("dvision/kalman/processNoiseCov", parameters.kalman.processNoiseCov);
    // GPARAM("dvision/kalman/measurementNoiseCov", parameters.kalman.measurementNoiseCov);
    // GPARAM("dvision/kalman/errorCovPost", parameters.kalman.errorCovPost);

    // Get object detector parameters
    GPARAM("dvision/object_detector/enable", parameters.object.enable);
    GPARAM("dvision/object_detector/showAllDetections", parameters.object.showAllDetections);
    GPARAM("dvision/object_detector/showBall", parameters.object.showBall);
    GPARAM("dvision/object_detector/showGoal", parameters.object.showGoal);
    GPARAM("dvision/object_detector/showObstacle", parameters.object.showObstacle);
    GPARAM("dvision/object_detector/home_folder", parameters.object.home_folder);
    GPARAM("dvision/object_detector/label_file", parameters.object.label_file);
    GPARAM("dvision/object_detector/net_cfg", parameters.object.net_cfg);
    GPARAM("dvision/object_detector/weight_file", parameters.object.weight_file);
    GPARAM("dvision/object_detector/low_thresh", parameters.object.low_thresh);
    GPARAM("dvision/object_detector/high_thresh", parameters.object.high_thresh);
    GPARAM("dvision/object_detector/ball_max_scale_coff", parameters.object.ball_max_scale_coff);
    GPARAM("dvision/object_detector/ball_wh_low_ratio", parameters.object.ball_wh_low_ratio);
    GPARAM("dvision/object_detector/ball_wh_high_ratio", parameters.object.ball_wh_high_ratio);

    // Get obstacle detector parameters
    GPARAM("dvision/obstacle_detector/enable", parameters.obstacle.enable);
    GPARAM("dvision/obstacle_detector/showAllObstacles", parameters.obstacle.showAllObstacles);
    GPARAM("dvision/obstacle_detector/showResObstacles", parameters.obstacle.showResObstacles);
    GPARAM("dvision/obstacle_detector/active", parameters.obstacle.active);
    GPARAM("dvision/obstacle_detector/h0", parameters.obstacle.h0);
    GPARAM("dvision/obstacle_detector/h1", parameters.obstacle.h1);
    GPARAM("dvision/obstacle_detector/s0", parameters.obstacle.s0);
    GPARAM("dvision/obstacle_detector/s1", parameters.obstacle.s1);
    GPARAM("dvision/obstacle_detector/v0", parameters.obstacle.v0);
    GPARAM("dvision/obstacle_detector/v1", parameters.obstacle.v1);
    GPARAM("dvision/obstacle_detector/decayConfidence", parameters.obstacle.decayConfidence);
    GPARAM("dvision/obstacle_detector/minValidConfidence", parameters.obstacle.minValidConfidence);
    GPARAM("dvision/obstacle_detector/lowPassCoef", parameters.obstacle.lowPassCoef);
    GPARAM("dvision/obstacle_detector/maxPossibleJump", parameters.obstacle.maxPossibleJump);
    GPARAM("dvision/obstacle_detector/dilate_1", parameters.obstacle.dilate_1);
    GPARAM("dvision/obstacle_detector/erode_1", parameters.obstacle.erode_1);
    GPARAM("dvision/obstacle_detector/dilate_2", parameters.obstacle.dilate_2);
    GPARAM("dvision/obstacle_detector/erode_2", parameters.obstacle.erode_2);
    GPARAM("dvision/obstacle_detector/maxDistance", parameters.obstacle.maxDistance);
    GPARAM("dvision/obstacle_detector/minArea", parameters.obstacle.minArea);
    GPARAM("dvision/obstacle_detector/minDistance", parameters.obstacle.minDistance);

    // Get ball detector parameters
    GPARAM("dvision/ball_detector/enable", parameters.ball.enable);
    GPARAM("dvision/ball_detector/showResult", parameters.ball.showResult);
    GPARAM("dvision/ball_detector/useKalman", parameters.ball.useKalman);
    GPARAM("dvision/ball_detector/useSimpleBlobDetecor", parameters.ball.useSimpleBlobDetecor);
    GPARAM("dvision/ball_detector/BBoxScale", parameters.ball.BBoxScale);
    GPARAM("dvision/ball_detector/thresholdStep", parameters.ball.thresholdStep);
    GPARAM("dvision/ball_detector/minThreshold", parameters.ball.minThreshold);
    GPARAM("dvision/ball_detector/maxThreshold", parameters.ball.maxThreshold);
    GPARAM("dvision/ball_detector/minRepeatability", parameters.ball.minRepeatability);
    GPARAM("dvision/ball_detector/filterByColor", parameters.ball.filterByColor);
    GPARAM("dvision/ball_detector/blobColor", parameters.ball.blobColor);
    GPARAM("dvision/ball_detector/filterByArea", parameters.ball.filterByArea);
    GPARAM("dvision/ball_detector/minArea", parameters.ball.minArea);
    GPARAM("dvision/ball_detector/maxArea", parameters.ball.maxArea);
    GPARAM("dvision/ball_detector/filterByCircularity", parameters.ball.filterByCircularity);
    GPARAM("dvision/ball_detector/minCircularity", parameters.ball.minCircularity);
    GPARAM("dvision/ball_detector/filterByConvexity", parameters.ball.filterByConvexity);
    GPARAM("dvision/ball_detector/minConvexity", parameters.ball.minConvexity);
    GPARAM("dvision/ball_detector/filterByInertia", parameters.ball.filterByInertia);
    GPARAM("dvision/ball_detector/minInertiaRatio", parameters.ball.minInertiaRatio);
    GPARAM("dvision/ball_detector/maxBoxCenterOffset", parameters.ball.maxBoxCenterOffset);
    GPARAM("dvision/ball_detector/useRadiusCheck", parameters.ball.useRadiusCheck);
    GPARAM("dvision/ball_detector/minBallRadiusRatio", parameters.ball.minBallRadiusRatio);
    GPARAM("dvision/ball_detector/maxBallRadiusRatio", parameters.ball.maxBallRadiusRatio);
    GPARAM("dvision/ball_detector/useInFieldCheck", parameters.ball.useInFieldCheck);
    GPARAM("dvision/ball_detector/minBallToFieldDist", parameters.ball.minBallToFieldDist);
    GPARAM("dvision/ball_detector/useDistCheck", parameters.ball.useDistCheck);
    GPARAM("dvision/ball_detector/maxSeeBallDist", parameters.ball.maxSeeBallDist);

    // Get circle detector parameters
    GPARAM("dvision/circle_detector/enable", parameters.circle.enable);
    GPARAM("dvision/circle_detector/minLineLen", parameters.circle.minLineLen);
    GPARAM("dvision/circle_detector/maxLineLen", parameters.circle.maxLineLen);
    GPARAM("dvision/circle_detector/maxDistBetween2LS", parameters.circle.maxDistBetween2LS);
    GPARAM("dvision/circle_detector/radiusMaxCoef", parameters.circle.radiusMaxCoef);
    GPARAM("dvision/circle_detector/radiusMinCoef", parameters.circle.radiusMinCoef);
    GPARAM("dvision/circle_detector/confiusedDist", parameters.circle.confiusedDist);
    GPARAM("dvision/circle_detector/minLineSegmentCount", parameters.circle.minLineSegmentCount);

    // Get field detector parameters
    GPARAM("dvision/field_detector/enable", parameters.field.enable);
    GPARAM("dvision/field_detector/showMask", parameters.field.showMask);
    GPARAM("dvision/field_detector/showResult", parameters.field.showResult);
    GPARAM("dvision/field_detector/showDebug", parameters.field.showDebug);
    GPARAM("dvision/field_detector/h0", parameters.field.h0);
    GPARAM("dvision/field_detector/h1", parameters.field.h1);
    GPARAM("dvision/field_detector/s0", parameters.field.s0);
    GPARAM("dvision/field_detector/s1", parameters.field.s1);
    GPARAM("dvision/field_detector/v0", parameters.field.v0);
    GPARAM("dvision/field_detector/v1", parameters.field.v1);
    GPARAM("dvision/field_detector/d_h0", parameters.field.d_h0);
    GPARAM("dvision/field_detector/d_h1", parameters.field.d_h1);
    GPARAM("dvision/field_detector/d_s0", parameters.field.d_s0);
    GPARAM("dvision/field_detector/d_s1", parameters.field.d_s1);
    GPARAM("dvision/field_detector/d_v0", parameters.field.d_v0);
    GPARAM("dvision/field_detector/d_v1", parameters.field.d_v1);
    GPARAM("dvision/field_detector/active", parameters.field.active);
    GPARAM("dvision/field_detector/erode", parameters.field.erode);
    GPARAM("dvision/field_detector/dilate", parameters.field.dilate);
    GPARAM("dvision/field_detector/erode2", parameters.field.erode2);
    GPARAM("dvision/field_detector/dilate2", parameters.field.dilate2);
    GPARAM("dvision/field_detector/maxContourCount", parameters.field.maxContourCount);
    GPARAM("dvision/field_detector/minArea", parameters.field.minArea);
    GPARAM("dvision/field_detector/changePitch", parameters.field.changePitch);
    GPARAM("dvision/field_detector/maxDownDiffPixelUp", parameters.field.maxDownDiffPixelUp);
    GPARAM("dvision/field_detector/maxDownDiffPixelDown", parameters.field.maxDownDiffPixelDown);
    GPARAM("dvision/field_detector/approxPoly", parameters.field.approxPoly);
    GPARAM("dvision/field_detector/maxAcceptDistance", parameters.field.maxAcceptDistance);
    GPARAM("dvision/field_detector/minAcceptX", parameters.field.minAcceptX);

    // Get goal detector parameters
    GPARAM("dvision/goal_detector/enable", parameters.goal.enable);
    GPARAM("dvision/goal_detector/showMask", parameters.goal.showMask);
    GPARAM("dvision/goal_detector/showHoughLines", parameters.goal.showHoughLines);
    GPARAM("dvision/goal_detector/showAllLines", parameters.goal.showAllLines);
    GPARAM("dvision/goal_detector/showResLine", parameters.goal.showResLine);
    GPARAM("dvision/goal_detector/showVote", parameters.goal.showVote);
    GPARAM("dvision/goal_detector/showExtendPoints", parameters.goal.showExtendPoints);
    GPARAM("dvision/goal_detector/useKalman", parameters.goal.useKalman);
    GPARAM("dvision/goal_detector/h0", parameters.goal.h0);
    GPARAM("dvision/goal_detector/h1", parameters.goal.h1);
    GPARAM("dvision/goal_detector/s0", parameters.goal.s0);
    GPARAM("dvision/goal_detector/s1", parameters.goal.s1);
    GPARAM("dvision/goal_detector/v0", parameters.goal.v0);
    GPARAM("dvision/goal_detector/v1", parameters.goal.v1);
    GPARAM("dvision/goal_detector/active", parameters.goal.active);
    GPARAM("dvision/goal_detector/MinLineLength", parameters.goal.MinLineLength);
    GPARAM("dvision/goal_detector/MaxLineGap", parameters.goal.MaxLineGap);
    GPARAM("dvision/goal_detector/MaxOutField", parameters.goal.MaxOutField);
    GPARAM("dvision/goal_detector/MinOutField", parameters.goal.MinOutField);
    GPARAM("dvision/goal_detector/OutFieldDistanceNear", parameters.goal.OutFieldDistanceNear);
    GPARAM("dvision/goal_detector/OutFieldDistanceFar", parameters.goal.OutFieldDistanceFar);
    GPARAM("dvision/goal_detector/OutFieldOffsetNear", parameters.goal.OutFieldOffsetNear);
    GPARAM("dvision/goal_detector/OutFieldOffsetMid", parameters.goal.OutFieldOffsetMid);
    GPARAM("dvision/goal_detector/OutFieldOffsetFar", parameters.goal.OutFieldOffsetFar);
    GPARAM("dvision/goal_detector/MinNearFieldUpPoint", parameters.goal.MinNearFieldUpPoint);
    GPARAM("dvision/goal_detector/DistanceToMerge", parameters.goal.DistanceToMerge);
    GPARAM("dvision/goal_detector/AngleToMerge", parameters.goal.AngleToMerge);
    GPARAM("dvision/goal_detector/CollinearLengthToMerge", parameters.goal.CollinearLengthToMerge);
    GPARAM("dvision/goal_detector/NearestDistance", parameters.goal.NearestDistance);
    GPARAM("dvision/goal_detector/FurthestDistance", parameters.goal.FurthestDistance);
    GPARAM("dvision/goal_detector/NearMinLen", parameters.goal.NearMinLen);
    GPARAM("dvision/goal_detector/NearMaxLen", parameters.goal.NearMaxLen);
    GPARAM("dvision/goal_detector/FarMinLen", parameters.goal.FarMinLen);
    GPARAM("dvision/goal_detector/FarMaxLen", parameters.goal.FarMaxLen);
    GPARAM("dvision/goal_detector/maxDistFromRobot", parameters.goal.maxDistFromRobot);
    GPARAM("dvision/goal_detector/jumpMax", parameters.goal.jumpMax);
    GPARAM("dvision/goal_detector/jumpDistanceNear", parameters.goal.jumpDistanceNear);
    GPARAM("dvision/goal_detector/jumpDistanceFar", parameters.goal.jumpDistanceFar);
    GPARAM("dvision/goal_detector/jumpDoubleNear", parameters.goal.jumpDoubleNear);
    GPARAM("dvision/goal_detector/jumpDoubleMid", parameters.goal.jumpDoubleMid);
    GPARAM("dvision/goal_detector/jumpDoubleFar", parameters.goal.jumpDoubleFar);
    GPARAM("dvision/goal_detector/doubleVote", parameters.goal.doubleVote);
    GPARAM("dvision/goal_detector/minDoubleLength", parameters.goal.minDoubleLength);
    GPARAM("dvision/goal_detector/minContinuesColor", parameters.goal.minContinuesColor);
    GPARAM("dvision/goal_detector/useGoalPostExtend", parameters.goal.useGoalPostExtend);
    GPARAM("dvision/goal_detector/extLengthPerAttempt", parameters.goal.extLengthPerAttempt);
    GPARAM("dvision/goal_detector/extInvalidPoints", parameters.goal.extInvalidPoints);
    GPARAM("dvision/goal_detector/extTotalPoints", parameters.goal.extTotalPoints);
    GPARAM("dvision/goal_detector/extDownMaxGap", parameters.goal.extDownMaxGap);
    GPARAM("dvision/goal_detector/extValidLength", parameters.goal.extValidLength);
    GPARAM("dvision/goal_detector/cutOffInvalidPoints", parameters.goal.cutOffInvalidPoints);
    GPARAM("dvision/goal_detector/useGoalLengthCheck", parameters.goal.useGoalLengthCheck);
    GPARAM("dvision/goal_detector/validGoalLengthCoff", parameters.goal.validGoalLengthCoff);
    GPARAM("dvision/goal_detector/useGoalWidthCheck", parameters.goal.useGoalWidthCheck);
    GPARAM("dvision/goal_detector/minGoalWidthRatio", parameters.goal.minGoalWidthRatio);
    GPARAM("dvision/goal_detector/maxGoalWidthRatio", parameters.goal.maxGoalWidthRatio);
    GPARAM("dvision/goal_detector/maxUnknownDistError", parameters.goal.maxUnknownDistError);
    GPARAM("dvision/goal_detector/useDarknetCheck", parameters.goal.useDarknetCheck);
    GPARAM("dvision/goal_detector/bboxScale", parameters.goal.bboxScale);
    GPARAM("dvision/goal_detector/minDarknetResultDist", parameters.goal.minDarknetResultDist);

    // Get lne detector parameters
    GPARAM("dvision/line_detector/enable", parameters.line.enable);
    GPARAM("dvision/line_detector/showUnmerged", parameters.line.showUnmerged);
    GPARAM("dvision/line_detector/showMask", parameters.line.showMask);
    GPARAM("dvision/line_detector/showResult", parameters.line.showResult);
    GPARAM("dvision/line_detector/showAllLine", parameters.line.showAllLine);
    GPARAM("dvision/line_detector/showVote", parameters.line.showVote);
    GPARAM("dvision/line_detector/showCanny", parameters.line.showCanny);
    GPARAM("dvision/line_detector/h0", parameters.line.h0);
    GPARAM("dvision/line_detector/h1", parameters.line.h1);
    GPARAM("dvision/line_detector/s0", parameters.line.s0);
    GPARAM("dvision/line_detector/s1", parameters.line.s1);
    GPARAM("dvision/line_detector/v0", parameters.line.v0);
    GPARAM("dvision/line_detector/v1", parameters.line.v1);
    GPARAM("dvision/line_detector/black_h0", parameters.line.black_h0);
    GPARAM("dvision/line_detector/black_h1", parameters.line.black_h1);
    GPARAM("dvision/line_detector/black_s0", parameters.line.black_s0);
    GPARAM("dvision/line_detector/black_s1", parameters.line.black_s1);
    GPARAM("dvision/line_detector/black_v0", parameters.line.black_v0);
    GPARAM("dvision/line_detector/black_v1", parameters.line.black_v1);
    GPARAM("dvision/line_detector/black_dilate", parameters.line.black_dilate);
    GPARAM("dvision/line_detector/active", parameters.line.active);
    GPARAM("dvision/line_detector/MinLineLength", parameters.line.MinLineLength);
    GPARAM("dvision/line_detector/AngleToMerge", parameters.line.AngleToMerge);
    GPARAM("dvision/line_detector/DistanceToMerge_0", parameters.line.DistanceToMerge_0);
    GPARAM("dvision/line_detector/DistanceToMerge_15", parameters.line.DistanceToMerge_15);
    GPARAM("dvision/line_detector/DistanceToMerge_30", parameters.line.DistanceToMerge_30);
    GPARAM("dvision/line_detector/DistanceToMerge_45", parameters.line.DistanceToMerge_45);
    GPARAM("dvision/line_detector/DistanceToMerge_60", parameters.line.DistanceToMerge_60);
    GPARAM("dvision/line_detector/DistanceToMerge_75", parameters.line.DistanceToMerge_75);
    GPARAM("dvision/line_detector/CollinearLengthToMerge", parameters.line.CollinearLengthToMerge);
    GPARAM("dvision/line_detector/maxLineGapHough", parameters.line.maxLineGapHough);
    GPARAM("dvision/line_detector/rhoHough", parameters.line.rhoHough);
    GPARAM("dvision/line_detector/thetaHough", parameters.line.thetaHough);
    GPARAM("dvision/line_detector/thresholdHough", parameters.line.thresholdHough);
    GPARAM("dvision/line_detector/jumpMax", parameters.line.jumpMax);
    GPARAM("dvision/line_detector/jumpMin", parameters.line.jumpMin);
    GPARAM("dvision/line_detector/widthCheck", parameters.line.widthCheck);
    GPARAM("dvision/line_detector/aprxDist", parameters.line.aprxDist);
    GPARAM("dvision/line_detector/doubleVote", parameters.line.doubleVote);
    GPARAM("dvision/line_detector/greenVote", parameters.line.greenVote);
    GPARAM("dvision/line_detector/colorVote", parameters.line.colorVote);
    GPARAM("dvision/line_detector/doubleVUse", parameters.line.doubleVUse);
    GPARAM("dvision/line_detector/greenVUse", parameters.line.greenVUse);
    GPARAM("dvision/line_detector/colorVUse", parameters.line.colorVUse);
    GPARAM("dvision/line_detector/doubleVStart", parameters.line.doubleVStart);
    GPARAM("dvision/line_detector/greenVStart", parameters.line.greenVStart);
    GPARAM("dvision/line_detector/colorVStart", parameters.line.colorVStart);
    GPARAM("dvision/line_detector/doubleVEnd", parameters.line.doubleVEnd);
    GPARAM("dvision/line_detector/greenVEnd", parameters.line.greenVEnd);
    GPARAM("dvision/line_detector/colorVEnd", parameters.line.colorVEnd);
    GPARAM("dvision/line_detector/cannyThreadshold_0", parameters.line.cannyThreadshold_0);
    GPARAM("dvision/line_detector/cannyThreadshold_15", parameters.line.cannyThreadshold_15);
    GPARAM("dvision/line_detector/cannyThreadshold_30", parameters.line.cannyThreadshold_30);
    GPARAM("dvision/line_detector/cannyThreadshold_45", parameters.line.cannyThreadshold_45);
    GPARAM("dvision/line_detector/cannyThreadshold_60", parameters.line.cannyThreadshold_60);
    GPARAM("dvision/line_detector/cannyThreadshold_75", parameters.line.cannyThreadshold_75);
    // GPARAM("dvision/line_detector/blurSize1", parameters.line.blurSize1);
    // GPARAM("dvision/line_detector/blurSize2", parameters.line.blurSize2);
    GPARAM("dvision/line_detector/blurSizeUp_1", parameters.line.blurSizeUp_1);
    // GPARAM("dvision/line_detector/blurSizeUp_2", parameters.line.blurSizeUp_2);
    GPARAM("dvision/line_detector/blurSizeDown_1", parameters.line.blurSizeDown_1);
    // GPARAM("dvision/line_detector/blurSizeDown_2", parameters.line.blurSizeDown_2);
    GPARAM("dvision/line_detector/blurSizeAll", parameters.line.blurSizeAll);
    GPARAM("dvision/line_detector/blurSplitRatio_0", parameters.line.blurSplitRatio_0);
    GPARAM("dvision/line_detector/blurSplitEndPitch", parameters.line.blurSplitEndPitch);
    GPARAM("dvision/line_detector/cannyaperture", parameters.line.cannyaperture);
    GPARAM("dvision/line_detector/lineAwayFromField", parameters.line.lineAwayFromField);


    // Get line_classifier detector parameters
    GPARAM("dvision/line_classifier/enable", parameters.line_classifier.enable);
    GPARAM("dvision/line_classifier/angle2HorLine", parameters.line_classifier.angle2HorLine);
    GPARAM("dvision/line_classifier/angle2VerLine", parameters.line_classifier.angle2VerLine);
    GPARAM("dvision/line_classifier/minLineLen", parameters.line_classifier.minLineLen);
    GPARAM("dvision/line_classifier/maxDistBothGoal", parameters.line_classifier.maxDistBothGoal);
    GPARAM("dvision/line_classifier/maxDistSingleGoal", parameters.line_classifier.maxDistSingleGoal);
    GPARAM("dvision/line_classifier/yawCorrectNum", parameters.line_classifier.yawCorrectNum);
    GPARAM("dvision/line_classifier/goalLineLen", parameters.line_classifier.goalLineLen);
    GPARAM("dvision/line_classifier/robotMaxDist2GoalLine", parameters.line_classifier.robotMaxDist2GoalLine);

    // Get field parameters
    GPARAM("dvision/field_model/field_length", parameters.field_model.field_length);
    GPARAM("dvision/field_model/field_width", parameters.field_model.field_width);
    GPARAM("dvision/field_model/goal_depth", parameters.field_model.goal_depth);
    GPARAM("dvision/field_model/goal_width", parameters.field_model.goal_width);
    GPARAM("dvision/field_model/goal_height", parameters.field_model.goal_height);
    GPARAM("dvision/field_model/goal_area_length", parameters.field_model.goal_area_length);
    GPARAM("dvision/field_model/goal_area_width", parameters.field_model.goal_area_width);
    GPARAM("dvision/field_model/penalty_mark_distance", parameters.field_model.penalty_mark_distance);
    GPARAM("dvision/field_model/center_circle_diameter", parameters.field_model.center_circle_diameter);
    GPARAM("dvision/field_model/border_strip_width", parameters.field_model.border_strip_width);
    GPARAM("dvision/field_model/ball_diameter", parameters.field_model.ball_diameter);

    // Get monitor parameters
    GPARAM("dvision/monitor/use_cv_show", parameters.monitor.use_cv_show);
    GPARAM("dvision/monitor/transmit_raw_img", parameters.monitor.transmit_raw_img);
    GPARAM("dvision/monitor/update_gui_img", parameters.monitor.update_gui_img);
    GPARAM("dvision/monitor/update_canny_img", parameters.monitor.update_canny_img);
    GPARAM("dvision/monitor/update_field_binary", parameters.monitor.update_field_binary);
    GPARAM("dvision/monitor/update_goal_binary", parameters.monitor.update_goal_binary);
    GPARAM("dvision/monitor/update_line_binary", parameters.monitor.update_line_binary);
    GPARAM("dvision/monitor/update_obstacle_binary", parameters.monitor.update_obstacle_binary);

    // amcl config
    GPARAM("dvision/amcl/enable", parameters.amcl.enable);
    GPARAM("dvision/amcl/num_particles", parameters.amcl.num_particles);
    GPARAM("dvision/amcl/alpha_slow", parameters.amcl.alpha_slow);
    GPARAM("dvision/amcl/alpha_fast", parameters.amcl.alpha_fast);
    GPARAM("dvision/amcl/resample_interval", parameters.amcl.resample_interval);
    GPARAM("dvision/amcl/dist_threshold", parameters.amcl.dist_threshold);
    GPARAM("dvision/amcl/z_hit", parameters.amcl.z_hit);
    GPARAM("dvision/amcl/z_rand", parameters.amcl.z_rand);
}

#undef GPARAM

Parameters parameters;
} // namespace dvision
