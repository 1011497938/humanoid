#pragma once

#include "dcommon/dcommon.hpp"
class One_D_Filter
{
  public:
    One_D_Filter();

    One_D_Filter(ddouble_t R_);

    ddouble_t update(ddouble_t prediction, ddouble_t measurement);

  private:
    ddouble_t P;
    ddouble_t Q;
    ddouble_t R;
    ddouble_t K;

    ddouble_t estimated;
};
