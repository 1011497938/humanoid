#include "dmotion/GaitStateSupportLib/onedCtrl.hpp"

#include <cmath>

onedCtrl::onedCtrl(ddouble_t vmax, ddouble_t amax)
{
    m_vmax = vmax;
    m_amax = amax;
}

onedCtrl::~onedCtrl()
{
}

void
onedCtrl::oned_analysis(ddouble_t x_out[], const ddouble_t x_start[], const ddouble_t x_target[], const int num_left)
{
    if (num_left == 0) {
        x_out[0] = x_start[0];
        x_out[1] = x_start[1];
        return;
    }
    int cake;
    ddouble_t xf, xf_c, vx0, vxf;
    ddouble_t S1, S2, S3;
    ddouble_t time_left, dT;
    int z_flag, amin_flag;
    ddouble_t amax, vmax;
    amax = m_amax;
    vmax = m_vmax;
    z_flag = 1;
    amin_flag = 0;
    dT = 0.012;
    xf = x_target[0] - x_start[0];
    xf_c = xf;
    S3 = xf;
    vx0 = x_start[1];
    vxf = x_target[1];
    time_left = num_left * dT;

    if (xf_c < 0) {
        xf_c = -xf_c;
        xf = -xf;
        vx0 = -vx0;
        vxf = -vxf;
        z_flag = -z_flag;
    }

    // S1
    if (vxf >= vx0)
        S1 = (vxf * vxf - vx0 * vx0) / 2 / amax;
    else
        S1 = (vx0 * vx0 - vxf * vxf) / 2 / amax;
    // S2
    S2 = (vmax * vmax - vxf * vxf) / 2 / amax + (vmax * vmax - vx0 * vx0) / 2 / amax;

    if (xf_c >= S2)
        cake = 1;
    if (xf_c >= S1 && xf_c < S2)
        cake = 2;

    if (fabs((vxf - vx0) / amax) >= time_left) {
        S3 = ((vx0 - time_left * amax) + vx0) * time_left / 2;
        cake = 4;
        if (vxf < vx0) {
            vx0 = -vx0;
            vxf = -vxf;
            xf = -xf;
            xf_c = -xf_c;
            z_flag = -z_flag;
        }
    } else if (xf_c < S1) // cake = 3
    {
        cake = 2;
        vx0 = -vx0;
        vxf = -vxf;
        xf = -xf;
        xf_c = -xf_c;
        z_flag = -z_flag;
    }

    // S3 means how fast i can go in the range of amax
    ddouble_t ta, tb, tc, td, vx1;
    ta = (vmax - vx0) / amax;
    tb = (vmax - vxf) / amax;
    vx1 = (time_left * amax + vx0 + vxf) / 2;
    tc = (vx1 - vx0) / amax;
    td = (vx1 - vxf) / amax;
    // cake 1,2
    if (ta + tb <= time_left) {
        S3 = (S2 + (time_left - ta - tb) * vmax);
        if (xf_c >= S3)
            cake = 1;
    }

    else if (tc >= 0 && td >= 0) {
        S3 = ((vx1 * vx1 - (vx0 * vx0 + vxf * vxf) / 2) / amax);
        if (xf_c >= S3)
            cake = 2;
    }

    if (xf_c > S3)
        xf_c = S3;

    /// determine a
    ddouble_t tmp_b, tmp_a, tmp_c, amax_calculate;

    tmp_b = 2 * time_left * vx0 + 2 * +time_left * vxf - 4 * xf;
    tmp_a = time_left * time_left;
    tmp_c = -(vx0 - vxf) * (vx0 - vxf);

    // std::cout<<tmp_a<<" "<<tmp_b<<" "<<tmp_c<<std::endl;

    amax_calculate = (-tmp_b + sqrt(tmp_b * tmp_b - 4 * tmp_a * tmp_c)) / 2 / tmp_a;

    amax = amax_calculate;

    if (amax > m_amax)
        amax = m_amax;
    if (amax < 0.1) {
        amax = 0.1;
        amin_flag = 1;
    }
    /// output
    ddouble_t tmp_out[2] = { 0, 0 };

    if (cake == 1) {
        ddouble_t t1, t2, a;
        t1 = (vmax - vx0) / amax;
        t2 = (xf_c - S2) / vmax;

        if (dT <= t1) {
            a = amax;
            tmp_out[1] = vx0 + a * dT;
            tmp_out[0] = (vx0 + tmp_out[1]) * dT / 2;

        } else if (dT <= t1 + t2) {
            a = 0;
            tmp_out[1] = vmax;
            tmp_out[0] = (vx0 + vmax) * t1 / 2 + (dT - t1) * vmax;
        } else {
            a = -amax;
            tmp_out[1] = vmax + a * (dT - t1 - t2);
            tmp_out[0] = (vx0 + vmax) * t1 / 2 + (t2)*vmax + (vmax + tmp_out[1]) * (dT - t1 - t2) / 2;
        }
    }
    // case 2
    if (cake == 2) {
        ddouble_t t1, vx1, a;
        vx1 = sqrt(amax * xf_c + (vx0 * vx0 + vxf * vxf) / 2);
        t1 = (vx1 - vx0) / amax;
        if (dT <= t1) {
            a = amax;
            tmp_out[1] = vx0 + a * dT;
            tmp_out[0] = (vx0 + tmp_out[1]) * dT / 2;
        } else {
            a = -amax;
            tmp_out[1] = vx1 + a * (dT - t1);
            tmp_out[0] = (vx0 + vx1) * t1 / 2 + (vx1 + tmp_out[1]) * (dT - t1) / 2;
        }
    }
    if (cake == 4) {
        ddouble_t a;
        a = amax;
        tmp_out[1] = vx0 + a * dT;
        tmp_out[0] = (vx0 + tmp_out[1]) * dT / 2;
    }

    if (amin_flag) {
        x_out[0] = x_start[0];
        x_out[1] = 0;
    } else {
        if (z_flag == 1) {
            x_out[0] = x_start[0] + tmp_out[0];
            x_out[1] = 0 + tmp_out[1];
        } else {
            x_out[0] = x_start[0] - tmp_out[0];
            x_out[1] = 0 - tmp_out[1];
        }
    }
}
