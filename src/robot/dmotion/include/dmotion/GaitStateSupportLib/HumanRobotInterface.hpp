#pragma once
#include "dmotion/MotionData.hpp"
#include <string>

#include "dcommon/dcommon.hpp"
class I_HumanRobot
{
  public:
    virtual ~I_HumanRobot();

    virtual void runWalk(ddouble_t tsx, ddouble_t tsy, ddouble_t tst) = 0;
    virtual void getAngle_serial(const RobotCtrl targetCtrl, int dataArray[], const bool isExcute) = 0;
    virtual void doTxTask(int* motionData) = 0;
    virtual ddouble_t* curveCreate(ddouble_t start, ddouble_t mid, ddouble_t last, int num) = 0;
    virtual int loadGaitFile(const std::string gaitname, ddouble_t** data) = 0;
    virtual void doCrouchFromStand(const int stepnum_) = 0;
    virtual void doCrouchFromStandMotor(const int stepnum_) = 0;
    virtual void doStandFromCrouch(const int stepnum_) = 0;
    virtual void dofirstStep() = 0;
    virtual void staticEntry() = 0;
    virtual void staticExit() = 0;

    RobotCtrl m_robotCtrl;

  public:
    std::string m_robot_name;
    int m_robot_number;

    bool m_leftkick_flag;
    bool m_rightkick_flag;
    ddouble_t m_motor_k[MOTORNUM];
    int m_motor_zf[MOTORNUM];
    int m_motor_lb[MOTORNUM];
    int m_motor_ub[MOTORNUM];
    ddouble_t m_AnkleH_mid_l; // the stepheight up
    ddouble_t m_AnkleH_last_l; // the stepheight down
    ddouble_t m_AnkleH_mid_r;
    ddouble_t m_AnkleH_last_r;

  public: /* to do be private */
    /* options */
    // int m_oldTurning;
    // ddouble_t m_stepK;
    // /* compensate the turning angle */
    // ddouble_t m_theta_AnticlockZ,m_theta_ClockwiseF;

    // /**
    //  * gait data
    //  **/
    // ddouble_t m_g;
    // /* sample time */
    // ddouble_t m_dt;
    // /* the distance between two ankles (id 6 and id 13 motors) */
    // ddouble_t m_ankle_distance;
    // /* the stepheight up */
    // ddouble_t m_AnkleH_mid_l;
    // /* the stepheight down */
    // ddouble_t m_AnkleH_last_l;
    // ddouble_t m_AnkleH_mid_r;
    // ddouble_t m_AnkleH_last_r;
    // int    m_stepnum;
    // ddouble_t m_yzmp;
    // //---------                 ---------
    // //|       |        x        |       |
    // //|   .   |   .    |        |   .   |
    // //|   |   |   |    |        |   |   |
    // //----|----   |y<---        ----|----
    // //    |-------|                 |
    // //    | m_yzmp                  |
    // //    |-------------------------|
    // //          m_ankle_distance

    // ddouble_t m_cm_r;
    // ddouble_t m_cm_p;
    // ddouble_t m_cm_y;
    // ddouble_t m_cm_dx;
    // ddouble_t m_cm_dy;
    // /* m_cm_dx_k*forwardmovelength = cm offset in x's direction */
    // ddouble_t m_cm_dx_k;
    // /* m_cm_dy_lk*leftmovelength = cm offset in y's direction */
    // ddouble_t m_cm_dy_lk;
    // ddouble_t m_cm_dy_rk;
    // ddouble_t m_percent_x;
    // ddouble_t m_kickpercent;
    // /* like center of mass */
    // ddouble_t m_hipHeight;
    // /* the offset when step */
    // ddouble_t m_stepZero;
};
