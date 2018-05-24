#include "dmotion/GaitStateLib/GaitStateStandup.hpp"

GaitStateStandup::GaitStateStandup(I_HumanRobot* robot)
  : GaitStateBase(STANDUP, robot)
{
}

GaitStateStandup::~GaitStateStandup() = default;

void
GaitStateStandup::entry()
{
    robot->staticEntry();
}

void
GaitStateStandup::execute()
{
    ROS_DEBUG("Standup execute");
    robot->doStandFromCrouch(RobotPara::stepnum * 4);
}

void
GaitStateStandup::exit()
{
    robot->doCrouchFromStand(RobotPara::stepnum * 2); // crouch first
    robot->staticExit();
}
