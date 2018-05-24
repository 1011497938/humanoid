#pragma once
#include "dmotion/GaitStateLib/GaitStateBase.hpp"
#include <ros/ros.h>

class GaitStateCrouch : public GaitStateBase
{
  public:
    GaitStateCrouch(I_HumanRobot*);
    ~GaitStateCrouch();
    void execute() override;
    void entry() override;
    void exit() override;

private:
    ros::Time last_entry_timestamp_ = ros::Time(0);
};
