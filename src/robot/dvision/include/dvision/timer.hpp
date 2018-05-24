#pragma once

#include <ros/ros.h>
namespace dvision {

class Timer
{
  public:
    inline Timer()
    {
        restart();
    }

    inline void restart()
    {
        m_startTimestamp = ros::Time::now();
    }

    inline double elapsedMsec()
    {
        auto res = elapsedSec() * 1000;
        restart();
        return res;
    }

    inline double elapsedSec()
    {
        auto now = ros::Time::now();
        auto res = (now - m_startTimestamp).toSec();
        restart();
        return res;
    }

    inline double getElapsedMsec()
    {
        auto res = elapsedSec() * 1000;
        return res;
    }

    inline double getElapsedSec()
    {
        auto now = ros::Time::now();
        auto res = (now - m_startTimestamp).toSec();
        return res;
    }

  private:
    ros::Time m_startTimestamp;
};
}
