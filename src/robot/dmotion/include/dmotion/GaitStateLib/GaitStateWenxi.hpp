#pragma once

#include "dmotion/GaitStateLib/GaitStateBase.hpp"
class GaitStateWenxi : public GaitStateBase
{
  public:
    GaitStateWenxi(I_HumanRobot* robot);
    ~GaitStateWenxi();
    void execute() override;
};
