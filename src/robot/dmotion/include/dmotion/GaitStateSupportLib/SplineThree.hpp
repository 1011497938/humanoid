#pragma once
#include "dcommon/dcommon.hpp"
class SplineThree
{
  public:
    SplineThree(int* plotx, ddouble_t* ploty, int ask, int num);
    SplineThree(int* plotx, ddouble_t* ploty, int ask, int num, ddouble_t temp1, ddouble_t temp2);
    ~SplineThree();

    void Calculate();
    ddouble_t ReturnValue(ddouble_t plotxx);

  private:
    int* x;
    ddouble_t xx, *y, *a, *b, *a1, *b1, *h, *m;
    int choice;
    int n;
    ddouble_t _temp1, _temp2;
};