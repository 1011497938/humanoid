#include "dmotion/GaitStateLib/GaitStateCrouch.hpp"

GaitStateCrouch::GaitStateCrouch(I_HumanRobot* robot)
  : GaitStateBase(CROUCH, robot)
{
}

GaitStateCrouch::~GaitStateCrouch() = default;

void
GaitStateCrouch::entry()
{
    last_entry_timestamp_ = ros::Time::now();
    robot->staticEntry();
}

void
GaitStateCrouch::execute()
{
    ROS_DEBUG("Crouch execute");
//    auto elapsed = (ros::Time::now() - last_entry_timestamp_).toSec();
//    if (elapsed > 3)
//        robot->doCrouchFromStand(1);
//    else
    robot->doCrouchFromStand(RobotPara::stand2crouch_stepnum);
}

void
GaitStateCrouch::exit()
{
    robot->staticExit();
}
