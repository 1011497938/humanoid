#pragma once

#include "dcommon/dcommon.hpp"
// brief
//一维轨迹的生成类，用来规划摆动腿的生成轨迹，输入vmax和amax分别代表了最大速度和最大加速度。
//原理详情可以参考《全方位移动机器人运动控制及规划》.吴永海. 4.3 1D轨迹规划
class onedCtrl
{
  public:
    onedCtrl(ddouble_t vmax, ddouble_t amax);
    ~onedCtrl(void);
    void oned_analysis(ddouble_t x_out[], const ddouble_t x_start[], const ddouble_t x_target[], const int num_left);

  private:
    ddouble_t m_vmax;
    ddouble_t m_amax;
    // ddouble_t m_x_start[2];
    // ddouble_t m_x_target[2];
    // ddouble_t m_x_out[2];
    // int m_num_left;
};
