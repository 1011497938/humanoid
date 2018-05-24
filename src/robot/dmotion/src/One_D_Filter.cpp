#include "dmotion/One_D_Filter.hpp"
#include <iostream>
using namespace std;

One_D_Filter::One_D_Filter()
{
    P = 1;
    Q = 1;
    R = 1000;
    K = 1;
    estimated = 0;
}

One_D_Filter::One_D_Filter(ddouble_t R_)
{
    P = 1;
    Q = 1;
    R = R_;
    K = 1;
    estimated = 0;
}

ddouble_t
One_D_Filter::update(ddouble_t prediction, ddouble_t measurement)
{
    P += Q;
    K = P / (P + R);
    P -= K * P;
    estimated += K * (measurement - estimated) + prediction;

    return estimated;
}
