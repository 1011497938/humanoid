#pragma once
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "dmotion/MotionData.hpp"
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "dcommon/dcommon.hpp"
typedef boost::numeric::ublas::matrix<ddouble_t> boost_matrix;

using namespace std;

/* to do, just for runwalk to specify runkick state, to be fixed */
class GaitStateManager;
class RobotStatus;
class transitHub;

class HumanRobot : public I_HumanRobot
{
  public:
    HumanRobot(ros::NodeHandle* m_nh, transitHub* port, RobotStatus* rs, GaitStateManager* manager);
    ~HumanRobot();

    /*walk*/
    void doTxTask(int* motionData);
    void runWalk(ddouble_t tsx, ddouble_t tsy, ddouble_t tst);
    ddouble_t* curveCreate(ddouble_t start, ddouble_t mid, ddouble_t last, int num);
    void getAngle_serial(const RobotCtrl targetCtrl, int dataArray[], const bool isExcute);
    ddouble_t* get_Angle(const ddouble_t* tChest, const ddouble_t* tAnkle, const ddouble_t* tHand, const bool whleg);
    void getVxyf0(const ddouble_t tsx, ddouble_t vxy[]);
    /*static*/
    int loadGaitFile(const std::string gaitname, ddouble_t** data);
    void dofirstStep();
    void doCrouchFromStand(const int stepnum_);
    void doCrouchFromStandMotor(const int stepnum_);
    void doStandFromCrouch(const int stepnum_);
    void staticEntry();
    void staticExit();

  private:
    int firstStep_method;
    ddouble_t* firstStep_data[5];
    int firstStep_length;

  private:
    void readOptions();
    void Leg_up(ddouble_t rsx, ddouble_t rsy, ddouble_t rst);
    void data2q(const int data[], ddouble_t lq[], ddouble_t rq[]); // lq[6],rq[6],data[motornum]
    void q2data(int data[], const ddouble_t lq[], const ddouble_t rq[]);
    ddouble_t getThetaAmend(const ddouble_t tsx);
    ddouble_t getRightAnkleMidHigh(const ddouble_t tsx);
    ddouble_t getLeftAnkleMidHigh(const ddouble_t tsx);
    ddouble_t getTheta(ddouble_t v);

    boost_matrix rotx(ddouble_t roll);
    boost_matrix roty(ddouble_t pit);
    boost_matrix rotz(ddouble_t yaw);
    boost_matrix rot2Tx(ddouble_t roll, ddouble_t x, ddouble_t y, ddouble_t z);
    boost_matrix rot2Ty(ddouble_t pit, ddouble_t x, ddouble_t y, ddouble_t z);
    boost_matrix rot2Tz(ddouble_t yaw, ddouble_t x, ddouble_t y, ddouble_t z);
    boost_matrix rpy2r(ddouble_t roll, ddouble_t pit, ddouble_t yaw);
    boost_matrix rpy2t(ddouble_t roll, ddouble_t pit, ddouble_t yaw, ddouble_t x, ddouble_t y, ddouble_t z);
    boost_matrix Array2row(ddouble_t* t_array, int row);

  private:
    /* for plat control */
    ros::NodeHandle* m_nh;
    transitHub* m_port;
    RobotStatus* m_status;
    GaitStateManager* m_manager;

    boost_matrix m_motor_bodyR;

    ddouble_t curYaw, curPitch;
    ddouble_t desYaw, desPitch;

    initdataDebug initdata_;

    bool m_simulation;
    int m_simDelay;
};
