#pragma once
#include "dmotion/GaitStateSupportLib/comEstimator.hpp"
#include "dmotion/MotionData.hpp"
#include "dmotion/MotionShareData.hpp"
#include "dmotion/One_D_Filter.hpp"
#include "dmotion/GaitStateSupportLib/imu_filter.hpp"
#include "dmotion/GaitStateSupportLib/world_frame.hpp"
#include "dmotion/GaitStateSupportLib/stateless_orientation.hpp"
#include "dmotion/VecPos.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <deque>
#include <ros/ros.h>
#include "dcommon/dcommon.hpp"

class RobotStatus
{
  public:
    explicit RobotStatus(ros::NodeHandle* nh);
    ~RobotStatus();
    void readOptions();

    /* get infromation about robot status */
  public:
    angledataDebug getAngledata();
    GyroData getGdata();
    void setGdata(GyroData);
    void resetGyro();
    void updateDeltaDist(float dx, float dy, float dt);
    initdataDebug getMotorinit();
    initdataDebug getRawMotorInit();
    void updateEularAngle();
    VecPos getEularAngle(); // 弧度制
    stabilityStatus checkStableState();
    void setCompassData(CompassData temp);
    void update_compass_config(ddouble_t, ddouble_t, ddouble_t, ddouble_t, ddouble_t);
    ddouble_t getCurrentFieldAngle();
    CompassData getCompassData();
    comEstimator* m_comInfo;
    std::deque<VecPos> m_eular_deque;
    deltadataDebug checkDeltaDist();
    deltadataDebug getDelta();
    void initMotor();
    ddouble_t m_last_angle_z;

  public:
    stabilityStatus m_bStable;
    ddouble_t curyaw;
    angledataDebug m_angledata;
    //---add by yyj---
    angledataDebug m_angle_rpy;
    ImuFilter m_imu_filter;
    bool imu_initialized;
    int imu_init_cnt;
    int imu_prepare_time;

    ddouble_t imu_roll_bias;
    ddouble_t imu_pitch_bias;
    ddouble_t imu_yaw_bias;
   //----------------
    GyroData m_gypdata;
    offsetDebug m_offset;
    CompassData m_compassdata;

    initdataDebug m_motorini;
    initdataDebug raw_motorini;

    deltadataDebug m_deltaDist;
    std::vector<ddouble_t> k_tmp;
    bool m_isRun;

  private:
    ros::NodeHandle* m_nh;
    One_D_Filter compass_filter;
    int robotnumber;
    ddouble_t gyro_x_avg;
    ddouble_t gyro_y_avg;
    ddouble_t gyro_z_avg;
    std::deque<ddouble_t> gyro_x_sum;
    std::deque<ddouble_t> gyro_y_sum;
    std::deque<ddouble_t> gyro_z_sum;
    int gyro_cnt;
};
