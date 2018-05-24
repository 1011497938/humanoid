#pragma once
#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateManager;
class GaitStateKick : public GaitStateBase
{
  public:
    GaitStateKick(I_HumanRobot* robot, GaitStateManager* manager);
    ~GaitStateKick();

    void exit() override;
    void entry() override;
    void execute() override;
    void loadGaitFile() override;
    void leftKickEntry();
    void rightKickEntry();

    void setLeftKick();
    void setRightKick();

  private:
    GaitStateManager* manager;
    int lengthR_;
    int lengthL_;
    int length_;
    bool crouchBool_;
    ddouble_t* dataR_[16];
    ddouble_t* dataL_[16];
    bool m_leftKick = false;
};
