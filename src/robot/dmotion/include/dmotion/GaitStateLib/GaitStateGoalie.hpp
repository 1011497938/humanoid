#pragma once

#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateGoalie : public GaitStateBase
{
  public:
    GaitStateGoalie(I_HumanRobot* robot);
    ~GaitStateGoalie();
    void execute() override;
    void entry() override;
    void exit() override;
    void loadGaitFile() override;

    void setLeftGoalie();
    void setRightGoalie();

  private:
    void doLiftbothHand();
    void doGoalieMid();
    void doRecover();

    int length;
    ddouble_t* data[20];

    bool m_goalie_left;

    int m_stepnum;
    bool m_goalie_bool;
    int m_sleeptime;
};
