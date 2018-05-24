#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace dvision {

struct CameraParameters
{
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeff;

    cv::Size undistImageSize;
    cv::Mat undistCameraMatrix;
    cv::Mat undistDistCoeff;

    double fx;
    double fy;
    double cx;
    double cy;

    double undistCx;
    double undistCy;
    double centerInUndistX; // oringin center in undist
    double centerInUndistY;

    int width;
    int height;

    int undistWidth;
    int undistHeight;

    // 16 extrinsic parameters, see meaning in Matlab code: main.m
    std::vector<double> extrinsic_para;
    bool debug = false;
};

struct CalibParameters
{
    bool calibOdometer;
};

struct KalmanFilterParameters
{
    int maxMissSec;
    bool allowReset;
    // float processNoiseCov;
    // float measurementNoiseCov;
    // float errorCovPost;
};

struct ObjectDetectorParameters
{
    bool enable;
    bool showAllDetections;
    bool showBall;
    bool showGoal;
    bool showObstacle;
    std::string home_folder;
    std::string label_file;
    std::string net_cfg;
    std::string weight_file;
    float low_thresh;
    float high_thresh;
    float ball_max_scale_coff;
    float ball_wh_low_ratio;
    float ball_wh_high_ratio;
};

struct ObstacleDetectorParameters
{
    bool enable;
    bool showAllObstacles;
    bool showResObstacles;
    bool active;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    float decayConfidence;
    float minValidConfidence;
    float lowPassCoef;
    float maxPossibleJump;
    int dilate_1;
    int erode_1;
    int dilate_2;
    int erode_2;
    float maxDistance;
    int minArea;
    int minDistance;
};

struct BallDetectorParameters
{
    bool enable;
    bool showResult;
    bool useKalman;
    // params for simple blob detector
    bool useSimpleBlobDetecor;
    float BBoxScale;
    int thresholdStep;
    int minThreshold;
    int maxThreshold;
    int minRepeatability;
    bool filterByColor;
    int blobColor;
    bool filterByArea;
    int minArea;
    int maxArea;
    bool filterByCircularity;
    float minCircularity;
    bool filterByConvexity;
    float minConvexity;
    bool filterByInertia;
    float minInertiaRatio;
    float maxBoxCenterOffset;
    bool useRadiusCheck;
    float minBallRadiusRatio;
    float maxBallRadiusRatio;
    bool useInFieldCheck;
    float minBallToFieldDist;
    bool useDistCheck;
    float maxSeeBallDist;
};

struct CircleDetectorParameters
{
    bool enable;
    float minLineLen;
    float maxLineLen;
    float maxDistBetween2LS;
    float radiusMaxCoef;
    float radiusMinCoef;
    int confiusedDist;
    int minLineSegmentCount;
};

struct FieldDetectorParameters
{
    bool enable;
    bool showMask;
    bool showResult;
    bool showDebug;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    int d_h0;
    int d_h1;
    int d_s0;
    int d_s1;
    int d_v0;
    int d_v1;
    bool active;
    int erode;
    int dilate;
    int erode2;
    int dilate2;
    int maxContourCount;
    int minArea;
    double changePitch;
    int maxDownDiffPixelUp;
    int maxDownDiffPixelDown;
    float approxPoly;
    float maxAcceptDistance;
    float minAcceptX;
};

struct GoalDetectorParameters
{
    bool enable;
    bool showMask;
    bool showHoughLines;
    bool showAllLines;
    bool showResLine;
    bool showVote;
    bool showExtendPoints;
    bool useKalman;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    bool active;
    int MinLineLength;
    int MaxLineGap;
    int MaxOutField;
    int MinOutField;
    int OutFieldDistanceNear;
    int OutFieldDistanceFar;
    int OutFieldOffsetNear;
    int OutFieldOffsetMid;
    int OutFieldOffsetFar;
    int MinNearFieldUpPoint;
    float DistanceToMerge;
    float AngleToMerge;
    float CollinearLengthToMerge;
    float NearestDistance;
    float FurthestDistance;
    int NearMinLen;
    int NearMaxLen;
    int FarMinLen;
    int FarMaxLen;
    int maxDistFromRobot;
    int jumpMax;
    int jumpDistanceNear;
    int jumpDistanceFar;
    int jumpDoubleNear;
    int jumpDoubleMid;
    int jumpDoubleFar;
    int doubleVote;
    int minDoubleLength;
    int minContinuesColor;
    bool useGoalPostExtend;
    int extLengthPerAttempt;
    int extInvalidPoints;
    int extTotalPoints;
    int extDownMaxGap;
    int extValidLength;
    int cutOffInvalidPoints;
    bool useGoalLengthCheck;
    float validGoalLengthCoff;
    bool useGoalWidthCheck;
    float minGoalWidthRatio;
    float maxGoalWidthRatio;
    float maxUnknownDistError;
    bool useDarknetCheck;
    float bboxScale;
    float minDarknetResultDist;
};

struct LineDetectorParameters
{
    bool enable;
    bool showUnmerged;
    bool showMask;
    bool showResult;
    bool showAllLine;
    bool showVote;
    bool showCanny;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    int black_h0;
    int black_h1;
    int black_s0;
    int black_s1;
    int black_v0;
    int black_v1;
    int black_dilate;
    bool active;
    int MinLineLength;
    int AngleToMerge;
    int CollinearLengthToMerge;
    int DistanceToMerge_0;
    int DistanceToMerge_15;
    int DistanceToMerge_30;
    int DistanceToMerge_45;
    int DistanceToMerge_60;
    int DistanceToMerge_75;
    int maxLineGapHough;
    float rhoHough;
    int thetaHough;
    int thresholdHough;
    int jumpMax;
    int jumpMin;
    float widthCheck;
    bool aprxDist;
    int doubleVote;
    int greenVote;
    int colorVote;
    bool doubleVUse;
    bool greenVUse;
    bool colorVUse;
    float doubleVStart;
    float greenVStart;
    float colorVStart;
    float doubleVEnd;
    float greenVEnd;
    float colorVEnd;
    int cannyThreadshold_0;
    int cannyThreadshold_15;
    int cannyThreadshold_30;
    int cannyThreadshold_45;
    int cannyThreadshold_60;
    int cannyThreadshold_75;
    // int blurSize1;
    // int blurSize2;
    int blurSizeUp_1;
    // int blurSizeUp_2;
    int blurSizeDown_1;
    // int blurSizeDown_2;
    int blurSizeAll;
    double blurSplitRatio_0;
    double blurSplitEndPitch;
    int cannyaperture;
    int lineAwayFromField;
};

struct LineClassifierParameters
{
    bool enable;
    double angle2HorLine;
    double angle2VerLine;
    double minLineLen;
    int maxDistBothGoal;
    int maxDistSingleGoal;
    int yawCorrectNum;
    double goalLineLen;
    double robotMaxDist2GoalLine;
};

struct FieldModelParameters
{
    int field_length;
    int field_width;
    int goal_depth;
    int goal_width;
    int goal_height;
    int goal_area_length;
    int goal_area_width;
    int penalty_mark_distance;
    int center_circle_diameter;
    int border_strip_width;
    int ball_diameter;
};

struct MonitorParameters
{
    bool use_cv_show;
    bool transmit_raw_img;
    bool update_gui_img;
    bool update_canny_img;
    bool update_field_binary;
    bool update_goal_binary;
    bool update_line_binary;
    bool update_obstacle_binary;
};

struct HSVRange
{
    bool active;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
};

struct Amcl
{
    bool enable;
    double z_hit = 0.95;
    double z_rand = 0.05;
    double sigma_hit = 0.2;
    int max_occ_dist = 25;
    double dist_threshold = 50;
    double alpha_slow = 0.00001;
    double alpha_fast = 0.1;
    int resample_interval = 10;
    int num_particles = 300;
};

struct Parameters
{
    CalibParameters calib;
    ObjectDetectorParameters object;
    ObstacleDetectorParameters obstacle;
    BallDetectorParameters ball;
    CircleDetectorParameters circle;
    FieldDetectorParameters field;
    GoalDetectorParameters goal;
    LineDetectorParameters line;

    LineClassifierParameters line_classifier;
    CameraParameters camera;
    FieldModelParameters field_model;
    MonitorParameters monitor;

    KalmanFilterParameters kalman;
    Amcl amcl;

    std::string udpBroadcastAddress;
    int robotId;
    bool simulation;

    void init(ros::NodeHandle* nh);
    void update();

  private:
    ros::NodeHandle* m_nh;
};

extern Parameters parameters;
}
