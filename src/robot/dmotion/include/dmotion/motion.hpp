#pragma once

#include "misc/process.hpp"
using namespace dancer2050;

class Motion : public Process<Motion>
{
  public:
    Motion();
    ~Motion();
    void tick() override;
};
