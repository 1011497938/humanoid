#include "dmotion/GaitStateLib/GaitStateBase.hpp"

ros::NodeHandle* GaitStateBase::m_nh = nullptr;

void
GaitStateBase::readOptions()
{
    if (!m_nh->getParam("dmotion/robot/stepnum", stepNum)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/setupside_ankle_theta", m_setupside_ankle_theta)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/setupside_arm_theta", m_setupside_arm_theta)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }

    if (!m_nh->getParam("dmotion/robot/goalie_theta", m_goalie_theta)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/goalie_bool", m_goalie_bool)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/setupside_ankle_theta", m_setupside_ankle_theta)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }

    if (!m_nh->getParam("dmotion/robot/left_y_max", length_max_l)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/right_y_max", length_max_r)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/top_x_max", length_max_f)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
    if (!m_nh->getParam("dmotion/robot/back_x_max", length_max_b)) {
        ROS_FATAL("GaitStateBase get pararm error");
    }
}

void
GaitStateBase::loadGaitFile()
{
}

bool
GaitStateBase::operator==(const GaitStateType type)
{
    return this->type == type;
}

bool
GaitStateBase::operator!=(const GaitStateType type)
{
    return this->type != type;
}
