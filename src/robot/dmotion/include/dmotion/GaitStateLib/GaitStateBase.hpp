#pragma once
#include "dmsgs/ActionCommand.h"
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "dmotion/GaitStateType.hpp"
#include "dcommon/dcommon.hpp"
#include <ros/ros.h>
#include <string>
// using dmotion::ActionCommand::gait_type;

class GaitStateBase
{
  public:
    virtual void execute() = 0;
    virtual void entry()
    {
    }
    virtual void exit()
    {
    }

    bool operator==(const GaitStateType type);
    bool operator!=(const GaitStateType type);
    virtual ~GaitStateBase(){};

    ddouble_t m_gait_sx;
    ddouble_t m_gait_sy;
    ddouble_t m_gait_st;
    ddouble_t m_gait_angle;

    int stepNum;
    int m_robot_number;
    ddouble_t m_setupside_ankle_theta;
    ddouble_t m_setupside_arm_theta;
    ddouble_t m_goalie_theta;
    bool m_goalie_bool;
    static inline void set_nh(ros::NodeHandle* nh)
    {
        m_nh = nh;
    }

    void readOptions();
    virtual void loadGaitFile();

  protected:
    GaitStateBase(GaitStateType type, I_HumanRobot* robot)
      : m_gait_sx(0)
      , m_gait_sy(0)
      , m_gait_st(0)
      , m_gait_angle(0)
      , type(type)
      , robot(robot)
    {
        readOptions();
    }

    const GaitStateType type;
    I_HumanRobot* robot;
    static ros::NodeHandle* m_nh;

    ddouble_t length_max_l;
    ddouble_t length_max_r;
    ddouble_t length_max_f;
    ddouble_t length_max_b;
};
