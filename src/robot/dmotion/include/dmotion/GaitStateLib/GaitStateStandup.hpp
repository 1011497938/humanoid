#pragma once

#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateStandup : public GaitStateBase
{
  public:
    GaitStateStandup(I_HumanRobot* robot);
    ~GaitStateStandup();
    void execute() override;
    void entry() override;
    void exit() override;
};
