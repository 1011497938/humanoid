#include "dmotion/GaitStateLib/GaitStateWenxi.hpp"

GaitStateWenxi::GaitStateWenxi(I_HumanRobot* robot)
  : GaitStateBase(WENXI, robot)
{
}

GaitStateWenxi::~GaitStateWenxi() = default;

void
GaitStateWenxi::execute()
{
    m_gait_sy = (m_gait_sy < length_max_l) ? m_gait_sy : length_max_l;
    m_gait_sy = (m_gait_sy > length_max_r) ? m_gait_sy : length_max_r;
    m_gait_sx = (m_gait_sx < length_max_f) ? m_gait_sx : length_max_f;
    m_gait_sx = (m_gait_sx > length_max_b) ? m_gait_sx : length_max_b;
    robot->runWalk(m_gait_sx, m_gait_sy, m_gait_st);
}
