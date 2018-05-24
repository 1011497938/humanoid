#include "dmotion/GaitStateSupportLib/HumanRobot.hpp"
#include "dmotion/GaitStateLib/GaitStateBase.hpp"
#include "dmotion/GaitStateManager.hpp"
#include "dmotion/GaitStateSupportLib/SplineThree.hpp"
#include "dmotion/GaitStateSupportLib/onedCtrl.hpp"
#include "dmsgs/ActionCommand.h"
#include <fstream>
#include <ros/ros.h>

// FIXME(MWX): tons of memory leak...

#define epso 0.00001f
using namespace std;

HumanRobot::~HumanRobot() = default;

HumanRobot::HumanRobot(ros::NodeHandle* nh, transitHub* port, RobotStatus* rs, GaitStateManager* manager)
  : m_nh(nh)
  , m_port(port)
  , m_status(rs)
  , m_manager(manager)
  , m_motor_bodyR(boost::numeric::ublas::matrix<ddouble_t>(3, 3))
{
    readOptions();
    m_robotCtrl = RobotCtrl(0, 0, 0, DOUBLE_BASED);
    curYaw = curPitch = desYaw = desPitch = 0;
    m_leftkick_flag = 0;
    m_rightkick_flag = 0;
    firstStep_length = loadGaitFile("pvhipY", firstStep_data);
}

void
HumanRobot::readOptions()
{
    if (!m_nh->getParam("dmotion/robot/ah_ml_zero", m_AnkleH_mid_l))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/robot/ah_fl", m_AnkleH_last_l))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/robot/ah_mr_zero", m_AnkleH_mid_r))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/robot/ah_fr", m_AnkleH_last_r))
        ROS_FATAL("HumanRobot get param error");

    vector<ddouble_t> zf;
    vector<ddouble_t> k;
    vector<ddouble_t> lb;
    vector<ddouble_t> ub;
    if (!m_nh->getParam("dmotion/motor/zf", zf))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/motor/k", k))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/motor/lb", lb))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/motor/ub", ub))
        ROS_FATAL("HumanRobot get param error");

    for (int i = 0; i < MOTORNUM; i++) {
        m_motor_zf[i] = zf[i];
        int k_tmp = k[i];
        m_motor_lb[i] = lb[i];
        m_motor_ub[i] = ub[i];
        if (k_tmp == 4096) {
            m_motor_k[i] = k_tmp * 180.0 / M_PI / 360;
        } else if (k_tmp == 1024) {
            m_motor_k[i] = k_tmp * 180.0 / M_PI / 300;
        } else {
            throw runtime_error("mx? rx? or other fu** dynamixel motor? ");
        }
    }
    if(!m_nh->getParam("/ZJUDancer/MotionSimdelay", m_simDelay)) {
        ROS_FATAL("HumanRobot get param error");
    }

    if(!m_nh->getParam("/ZJUDancer/Simulation", m_simulation)) {
        ROS_FATAL("HumanRobot get param error");
    }
    ROS_INFO("Human Robot readOptions success");
}

int
HumanRobot::loadGaitFile(const std::string gaitname, ddouble_t** data)
{
    ROS_INFO("Loading gait: %s", gaitname.c_str());
    ROS_WARN("TODO(motion group) do not use raw array!!!!!!!!!!! why kick dataR_[i]'s addr is weird??");
    vector<ddouble_t> tmp;
    int row;
    if (!m_nh->getParam("dmotion/" + gaitname + "/data", tmp))
        ROS_FATAL("HumanRobot get param error");
    if (!m_nh->getParam("dmotion/" + gaitname + "/row", row))
        ROS_FATAL("HumanRobot get param error");
    int len = tmp.size() / row;
    for (int i = 0; i < row; ++i) {
        data[i] = new ddouble_t[len];
        for (int j = 0; j < len; ++j) {
            data[i][j] = tmp[i * len + j];
        }
    }
    return len;
}

boost_matrix
HumanRobot::rotx(ddouble_t roll)
{
    boost_matrix Rx(3, 3);
    Rx(0, 0) = 1;
    Rx(0, 1) = 0;
    Rx(0, 2) = 0;
    Rx(1, 0) = 0;
    Rx(1, 1) = cos(roll);
    Rx(1, 2) = -sin(roll);
    Rx(2, 0) = 0;
    Rx(2, 1) = sin(roll);
    Rx(2, 2) = cos(roll);
    return Rx;
}

boost_matrix
HumanRobot::roty(ddouble_t pit)
{
    boost_matrix Ry(3, 3);
    Ry(0, 0) = cos(pit);
    Ry(0, 1) = 0;
    Ry(0, 2) = sin(pit);
    Ry(1, 0) = 0;
    Ry(1, 1) = 1;
    Ry(1, 2) = 0;
    Ry(2, 0) = -sin(pit);
    Ry(2, 1) = 0;
    Ry(2, 2) = cos(pit);
    return Ry;
}

boost_matrix
HumanRobot::rotz(ddouble_t yaw)
{
    boost_matrix Rz(3, 3);
    Rz(0, 0) = cos(yaw);
    Rz(0, 1) = -sin(yaw);
    Rz(0, 2) = 0;
    Rz(1, 0) = sin(yaw);
    Rz(1, 1) = cos(yaw);
    Rz(1, 2) = 0;
    Rz(2, 0) = 0;
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;
    return Rz;
}

boost_matrix
HumanRobot::rot2Tx(ddouble_t roll, ddouble_t x, ddouble_t y, ddouble_t z)
{
    boost_matrix Rx(4, 4);
    Rx(0, 0) = 1;
    Rx(0, 1) = 0;
    Rx(0, 2) = 0;
    Rx(0, 3) = x;
    Rx(1, 0) = 0;
    Rx(1, 1) = cos(roll);
    Rx(1, 2) = -sin(roll);
    Rx(1, 3) = y;
    Rx(2, 0) = 0;
    Rx(2, 1) = sin(roll);
    Rx(2, 2) = cos(roll);
    Rx(2, 3) = z;
    Rx(3, 0) = 0;
    Rx(3, 1) = 0;
    Rx(3, 2) = 0;
    Rx(3, 3) = 1;
    return Rx;
}

boost_matrix
HumanRobot::rot2Ty(ddouble_t pit, ddouble_t x, ddouble_t y, ddouble_t z)
{
    boost_matrix Ry(4, 4);
    Ry(0, 0) = cos(pit);
    Ry(0, 1) = 0;
    Ry(0, 2) = sin(pit);
    Ry(0, 3) = x;
    Ry(1, 0) = 0;
    Ry(1, 1) = 1;
    Ry(1, 2) = 0;
    Ry(1, 3) = y;
    Ry(2, 0) = -sin(pit);
    Ry(2, 1) = 0;
    Ry(2, 2) = cos(pit);
    Ry(2, 3) = z;
    Ry(3, 0) = 0;
    Ry(3, 1) = 0;
    Ry(3, 2) = 0;
    Ry(3, 3) = 1;
    return Ry;
}

boost_matrix
HumanRobot::rot2Tz(ddouble_t yaw, ddouble_t x, ddouble_t y, ddouble_t z)
{
    boost_matrix Rz(4, 4);
    Rz(0, 0) = cos(yaw);
    Rz(0, 1) = -sin(yaw);
    Rz(0, 2) = 0;
    Rz(0, 3) = x;
    Rz(1, 0) = sin(yaw);
    Rz(1, 1) = cos(yaw);
    Rz(1, 2) = 0;
    Rz(1, 3) = y;
    Rz(2, 0) = 0;
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;
    Rz(2, 3) = z;
    Rz(3, 0) = 0;
    Rz(3, 1) = 0;
    Rz(3, 2) = 0;
    Rz(3, 3) = 1;
    return Rz;
}

boost_matrix
HumanRobot::rpy2t(ddouble_t roll, ddouble_t pit, ddouble_t yaw, ddouble_t x, ddouble_t y, ddouble_t z)
{
    //旋转的顺序是 yaw , pitch , roll
    boost_matrix R_tmp(4, 4);
    R_tmp = prod(rot2Tz(yaw, x, y, z), rot2Ty(pit, 0, 0, 0));
    return prod(R_tmp, rot2Tx(roll, 0, 0, 0));
}

boost_matrix
HumanRobot::rpy2r(ddouble_t roll, ddouble_t pit, ddouble_t yaw) /// ypr order in gyroscope
{
    //旋转的顺序是 yaw , pitch , roll
    boost_matrix R_tmp(3, 3);
    R_tmp = prod(rotz(yaw), roty(pit));
    return prod(R_tmp, rotx(roll));
}

/************************************************
 *robot functions add by davince 2014.10.9 \brief Array2row
 *transform the array to matrix row x 1
 * \param t_array length = row
 * \param row
 * \return matrix row x 1
 *
 ***********************************************/
boost_matrix
HumanRobot::Array2row(ddouble_t* t_array, int row)
{
    boost_matrix R_tmp((unsigned long)row, 1);
    for (int i = 0; i < row; i++)
        R_tmp((unsigned long)i, 0) = t_array[i];
    return R_tmp;
}

/************************************************
 *  robot functions add by davince 2014.10.9
 * \brief get_Angle
 * inverse kinetics
 * \param tChest Chest Postion (x,y,z,r,p,y)
 * \param tAnkle Ankle Position (x,y,z,r,p,y)
 * \param tHand Hand Position (x,y,z,r,p,y)
 * \param whleg left or right leg
 * \return *q 6x1 legs' angle (rad)
 *详细可见 《仿人机器人第》
 ***********************************************/
ddouble_t*
HumanRobot::get_Angle(const ddouble_t* tChest, const ddouble_t* tAnkle, const ddouble_t* tHand, const bool whleg)
{
    ddouble_t fChest_P[3], fChest_RPY[3], fAnkle_P[3], fAnkle_RPY[3];
    for (int i = 0; i < 3; i++) {
        fAnkle_P[i] = tAnkle[i];
        fChest_P[i] = tChest[i];
        fChest_RPY[i] = tChest[i + 3];
        fAnkle_RPY[i] = tAnkle[i + 3];
    }

    /// ROBOT CONFIG
    ddouble_t l1 = -RobotPara::upper_leg;
    ddouble_t l2 = -RobotPara::lower_leg;
    ddouble_t l3 = RobotPara::upper_arm;
    ddouble_t l4 = RobotPara::lower_arm;
    ddouble_t lby, lbz;
    lbz = -RobotPara::lbz;

    if (m_robotCtrl.supportStatus == DOUBLE_BASED) {
        if (whleg == 0) // 0 means rightLeg
            lby = -RobotPara::hip_distance / 2 - RobotPara::yzmp; //理解时可以先将yzmp 假想成0
        else
            lby = RobotPara::hip_distance / 2 + RobotPara::yzmp;
    } else {
        if (whleg == 0) // 0 means rightLeg
            lby = -RobotPara::hip_distance / 2 - RobotPara::yzmp;
        else
            lby = RobotPara::hip_distance / 2 + RobotPara::yzmp;
    }

    boost_matrix Lb(3, 1);
    Lb(0, 0) = 0;
    Lb(1, 0) = lby;
    Lb(2, 0) = lbz;

    ddouble_t* q = new ddouble_t[9];
    // (FIXME): MWX memory leak!!!!!!!!!!!!!!!!!!
    for (int i = 0; i < 9; i++)
        q[i] = 0;
//    boost_matrix Chest_R, Ankle_R, Chest_P, Ankle_P, Hip_P, r;
    auto Chest_R = rpy2r(fChest_RPY[0], fChest_RPY[1], fChest_RPY[2]);
    auto Chest_P = Array2row(fChest_P, 3);
    auto Ankle_R = rpy2r(fAnkle_RPY[0], fAnkle_RPY[1], fAnkle_RPY[2]);
    auto Ankle_P = Array2row(fAnkle_P, 3);
    auto Hip_P = Chest_P + prod(Chest_R, Lb);

    auto r = prod(trans(Ankle_R), Hip_P - Ankle_P);
    ddouble_t D = sqrt(pow(r(0, 0), 2) + pow(r(1, 0), 2) + pow(r(2, 0), 2));
    ddouble_t tmp_q3 = (D * D - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    if (tmp_q3 >= 1)
        q[3 - 1] = 0;
    else if (tmp_q3 <= -1)
        q[3 - 1] = M_PI;
    else
        q[3 - 1] = acos(tmp_q3);
    ddouble_t q6a = asin(l2 * sin(M_PI - q[3 - 1]) / D);
    q[2 - 1] = -atan2(r(0, 0), (r(2, 0) / abs(r(2, 0))) * sqrt(r(1, 0) * r(1, 0) + r(2, 0) * r(2, 0))) + q6a;
    q[1 - 1] = atan2(r(1, 0), r(2, 0));

    if (q[1 - 1] > 0.5 * M_PI)
        q[1 - 1] = q[1 - 1] - M_PI;
    else if (q[1 - 1] < -0.5 * M_PI)
        q[1 - 1] = q[1 - 1] + M_PI;

    boost_matrix tmp1, tmp2, RR;
    tmp1 = prod(trans(Chest_R), Ankle_R);
    tmp2 = prod(tmp1, rotx(-q[1 - 1]));
    RR = prod(tmp2, roty(-q[3 - 1] - q[2 - 1]));

    q[6 - 1] = atan2(-RR(0, 1), RR(1, 1)); // 髋关节yaw角z
    q[5 - 1] = atan2(RR(2, 1), -RR(0, 1) * sin(q[6 - 1]) + RR(1, 1) * cos(q[6 - 1])); // 髋关节rol角x
    q[4 - 1] = atan2(-RR(2, 0), RR(2, 2));

    /// for hand id 1 17 18(right hand) id 8 15 16(left
    /// hand)(TODO:解算手部逆运动学需重新推倒，
    //当前只是简单的平面二连杆解析解)
    q[7 - 1] = 0;
    q[8 - 1] = 0;
    q[9 - 1] = 0;
    ddouble_t x, z;
    ddouble_t thand0 = tHand[0];
    if (thand0 >= RobotPara::lower_arm + RobotPara::upper_arm)
        thand0 = RobotPara::lower_arm + RobotPara::upper_arm;

    x = tHand[0] * sin(tHand[1] * M_PI / 180);
    z = -tHand[0] * cos(tHand[1] * M_PI / 180);
    ddouble_t theta2, theta4, theta1, theta3;

    theta2 = M_PI - acos((l3 * l3 + l4 * l4 - x * x - z * z) / 2 / l3 / l4);

    theta4 = acos(-z / (sqrt(x * x + z * z)));
    if (x < 0) {
        theta4 = -theta4;
    }

    theta3 = acos((z * z + x * x + l3 * l3 - l4 * l4) / 2 / l3 / sqrt(x * x + z * z));

    theta1 = theta4 - theta3;
    // TODO::when there is a situation the current theta1 angle is 30degree and
    // next is -30
    // the motor will move 30->350 not 30->-30
    // template annotate it to solve
    // ***********
    // if (theta1 > M_PI) theta1 = theta1 - 2 * M_PI;
    // if (theta1 < -M_PI) theta1 = theta1 + 2 * M_PI;
    //**********
    q[7 - 1] = theta1;
    q[9 - 1] = theta2;
    return q;
}

/************************************************
 * \brief curveCreate
 * create spline3 or other curves
 * \param start: the start point of the curve
 * \param mid: the mid point of the curve if==NULL only start and last
 * \param last: the final point of the curve
 * \param num: the number of points
 * \return ddouble_t* x_s[num]
 *
 ***********************************************/
ddouble_t*
HumanRobot::curveCreate(ddouble_t start, ddouble_t mid, ddouble_t last, int num)
{
    ddouble_t* x_s = new ddouble_t[num];
    if (mid == false) {
        ddouble_t a = 1;
        int x_num[2];
        ddouble_t x_value[2];
        x_num[0] = 1;
        x_num[1] = (int)(num * a);
        x_value[0] = start;
        x_value[1] = last;
        SplineThree makerX = SplineThree(x_num, x_value, 1, 2);
        makerX.Calculate();
        for (int i = 0; i < num * a; i++)
            x_s[i] = makerX.ReturnValue(i + 1);
        for (int i = (int)(num * a - 1); i < num; i++)
            x_s[i] = x_value[1];
    } else {
        int z_num1[2], z_num2[2];
        ddouble_t z_value1[2], z_value2[2];
        z_num1[0] = 1;
        z_num1[1] = (num / 2);
        z_num2[0] = 1;
        z_num2[1] = num - z_num1[1];
        z_value1[0] = start;
        z_value1[1] = mid;
        z_value2[0] = mid;
        z_value2[1] = last;
        SplineThree makerZ1 = SplineThree(z_num1, z_value1, 1, 2);
        SplineThree makerZ2 = SplineThree(z_num2, z_value2, 1, 2);
        makerZ1.Calculate();
        makerZ2.Calculate();
        for (int i = 0; i < (num / 2); i++) {
            x_s[i] = makerZ1.ReturnValue(i + 1);
        }
        x_s[num / 2 - 1] = mid;
        for (int i = 0; i < z_num2[1]; i++) {
            x_s[i + (num / 2)] = makerZ2.ReturnValue(i + 1);
        }
        x_s[num - 1] = last;
    }
    return x_s;
}

/************************************************
 * \brief data2q
 * transform the data to lq[8] and rq[8]
 * \param data data[MOTORNUM] 20
 * \param lq lq[8]
 * \param rq rq[8]
 * \return lq and rq
 *
 ***********************************************/
void
HumanRobot::data2q(const int data[], ddouble_t lq[], ddouble_t rq[])
{
    lq[0] = data[14 - 1] / m_motor_k[14 - 1] * 1.0 / m_motor_zf[14 - 1];
    lq[1] = data[13 - 1] / m_motor_k[13 - 1] * 1.0 / m_motor_zf[13 - 1];
    lq[2] = data[12 - 1] / m_motor_k[12 - 1] * 1.0 / m_motor_zf[12 - 1];
    lq[3] = data[9 - 1] / m_motor_k[9 - 1] * 1.0 / m_motor_zf[9 - 1];
    lq[4] = data[10 - 1] / m_motor_k[10 - 1] * 1.0 / m_motor_zf[10 - 1];
    lq[5] = data[11 - 1] / m_motor_k[11 - 1] * 1.0 / m_motor_zf[11 - 1];

    lq[6] = data[8 - 1] / m_motor_k[8 - 1] * 1.0 / m_motor_zf[8 - 1]; // id 8,15,16
    lq[7] = data[15 - 1] / m_motor_k[15 - 1] * 1.0 / m_motor_zf[15 - 1];
    lq[8] = data[16 - 1] / m_motor_k[16 - 1] * 1.0 / m_motor_zf[16 - 1];

    rq[0] = data[7 - 1] / m_motor_k[7 - 1] * 1.0 / m_motor_zf[7 - 1];
    rq[1] = data[6 - 1] / m_motor_k[6 - 1] * 1.0 / m_motor_zf[6 - 1];
    rq[2] = data[5 - 1] / m_motor_k[5 - 1] * 1.0 / m_motor_zf[5 - 1];
    rq[3] = data[2 - 1] / m_motor_k[2 - 1] * 1.0 / m_motor_zf[2 - 1];
    rq[4] = data[3 - 1] / m_motor_k[3 - 1] * 1.0 / m_motor_zf[3 - 1];
    rq[5] = data[4 - 1] / m_motor_k[4 - 1] * 1.0 / m_motor_zf[4 - 1];

    rq[6] = data[1 - 1] / m_motor_k[1 - 1] * 1.0 / m_motor_zf[1 - 1]; // id 1,17,18
    rq[7] = data[17 - 1] / m_motor_k[17 - 1] * 1.0 / m_motor_zf[17 - 1];
    rq[8] = data[18 - 1] / m_motor_k[18 - 1] * 1.0 / m_motor_zf[18 - 1];
}

/************************************************
 * \brief q2data
 *transform lq[6] and rq[6] to data
 * \param data data[MOTORNUM] 20
 * \param lq lq[6]
 * \param rq rq[6]
 * \return data
 *
 ***********************************************/
void
HumanRobot::q2data(int data[], const ddouble_t lq[], const ddouble_t rq[])
{
    data[7 - 1] = (int)round(rq[0] * m_motor_k[7 - 1] * 1.0 * m_motor_zf[7 - 1]);
    data[6 - 1] = (int)round(rq[1] * m_motor_k[6 - 1] * 1.0 * m_motor_zf[6 - 1]);
    data[5 - 1] = (int)round(rq[2] * m_motor_k[5 - 1] * 1.0 * m_motor_zf[5 - 1]);
    data[2 - 1] = (int)round(rq[3] * m_motor_k[2 - 1] * 1.0 * m_motor_zf[2 - 1]);
    data[3 - 1] = (int)round(rq[4] * m_motor_k[3 - 1] * 1.0 * m_motor_zf[3 - 1]);
    data[4 - 1] = (int)round(rq[5] * m_motor_k[4 - 1] * 1.0 * m_motor_zf[4 - 1]);
    data[1 - 1] = (int)round(rq[6] * m_motor_k[1 - 1] * 1.0 * m_motor_zf[1 - 1]);
    data[17 - 1] = (int)round(rq[7] * m_motor_k[17 - 1] * 1.0 * m_motor_zf[17 - 1]);
    data[18 - 1] = (int)round(rq[8] * m_motor_k[18 - 1] * 1.0 * m_motor_zf[18 - 1]);

    data[14 - 1] = (int)round(lq[0] * m_motor_k[14 - 1] * 1.0 * m_motor_zf[14 - 1]);
    data[13 - 1] = (int)round(lq[1] * m_motor_k[13 - 1] * 1.0 * m_motor_zf[13 - 1]);
    data[12 - 1] = (int)round(lq[2] * m_motor_k[12 - 1] * 1.0 * m_motor_zf[12 - 1]);
    data[9 - 1] = (int)round(lq[3] * m_motor_k[9 - 1] * 1.0 * m_motor_zf[9 - 1]);
    data[10 - 1] = (int)round(lq[4] * m_motor_k[10 - 1] * 1.0 * m_motor_zf[10 - 1]);
    data[11 - 1] = (int)round(lq[5] * m_motor_k[11 - 1] * 1.0 * m_motor_zf[11 - 1]);
    data[8 - 1] = (int)round(lq[6] * m_motor_k[8 - 1] * 1.0 * m_motor_zf[8 - 1]);
    data[15 - 1] = (int)round(lq[7] * m_motor_k[15 - 1] * 1.0 * m_motor_zf[15 - 1]);
    data[16 - 1] = (int)round(lq[8] * m_motor_k[16 - 1] * 1.0 * m_motor_zf[16 - 1]);
}

/*************************************************
 * \brief doTxTask
 * send motiondata[20] to motors
 * \param motionData motionData[20] (motor special)
 * \return NULL
 *
 **************************************************/
void
HumanRobot::doTxTask(int* motionData)
{
    initdata_ = m_status->getMotorinit();
    if (motionData != NULL) {
        for (int i = 0; i < MOTORNUM; i++) {
            int sendData = motionData[i] + initdata_.initial[i];
            if (sendData > m_motor_ub[i]) {
                motionData[i] = m_motor_ub[i] - initdata_.initial[i];
            }
            if (sendData < m_motor_lb[i]) {
                motionData[i] = m_motor_lb[i] - initdata_.initial[i];
            }
            m_robotCtrl.dataArray[i] = motionData[i];
        }
    }

    int cameraData[2];
    m_manager->platCtrl(curYaw, curPitch);
    //printf("zf 18: %d 19: %d\n", m_motor_zf[18], m_motor_zf[19]);

    cameraData[0] = static_cast<int>((curPitch + RobotPara::diffV) * m_motor_k[18] * M_PI / 180 * m_motor_zf[18]);
    cameraData[1] = static_cast<int>((curYaw + RobotPara::diffH) * m_motor_k[19] * M_PI / 180 * m_motor_zf[19]);

    if(m_simulation) {
        usleep(m_simDelay);
    } else {
        m_port->doLoopTx(motionData, cameraData);
    }
}

/************************************************
 * \brief Leg_up
 * the main program in robot walking ,the robot
 * will take only one step(right foot) according to the inputs.
 * first ,using the zmp to generate the hip trajectory
 * second,using the footpos's change and curveCreate to generate the ankle
 *trajectory
 * third,using getAngle_serial if m_isSerial is true to get the angle of robot's
 *joints
 * forth,using doTxtasks to send angle to the motors
 *详细的原理可以参考《仿人机器人》4.3.3 三维步行模式生成
 *
 ************************************************/
void
HumanRobot::Leg_up(ddouble_t rsx, ddouble_t rsy, ddouble_t rst)
{
    ddouble_t tsx, tsy, tst; // command in robot

    ddouble_t lst_amd = getThetaAmend(rsx);
    tsx = rsx + RobotPara::step_x_amend;
    tsy = rsy;
    tst = rst + lst_amd;

    ddouble_t vxy[2];

    vector<ddouble_t> sx, sy, st;
    sx.push_back(m_robotCtrl.robot_x);
    sx.push_back(tsx);
    sx.push_back(tsx);

    getVxyf0(sx[0], vxy);

    sy.push_back(RobotPara::ankle_distance);
    if (m_robotCtrl.supportStatus == RIGHT_BASED) {
        sy.push_back(RobotPara::ankle_distance + tsy);
        sy.push_back(RobotPara::ankle_distance + tsy);
    } else // LEFT_BASED
    {
        sy.push_back(RobotPara::ankle_distance - tsy);
        sy.push_back(RobotPara::ankle_distance - tsy);
    }

    if (RobotPara::oldturning) {
        st.push_back(0);
        st.push_back(0);
        st.push_back(0);
    } else {
        st.push_back(0);
        st.push_back(tst);
        st.push_back(tst);
    }
    // cout<<"the size is"<<sx.size()<<endl;
    ddouble_t last_element = sx.size() - 1;
    if (sx.size() < 3) {
        sx.push_back(sx[last_element]);
        sx.push_back(sx[last_element]);
        sy.push_back(RobotPara::ankle_distance);
        sy.push_back(RobotPara::ankle_distance);
        st.push_back(st[last_element]);
        st.push_back(st[last_element]);
    }

    ddouble_t x_f0 = -sx[0] / 2;
    ddouble_t sy_te = sy[0] - RobotPara::ankle_distance;
    ddouble_t y_f0 = sy_te;

    for (vector<ddouble_t>::size_type i = 0; i != sy.size(); i++) {
        sy[i] = sy[i] + RobotPara::yzmp + RobotPara::yzmp;
        st[i] = st[i] / 180 * M_PI;
    }
    st[0] = 0;
    // cout<<"sxyt is "<<sx[0]<<" "<<sy[0]<<" "<<st[0]<<endl;
    // cout<<"sxyt is "<<sx[1]<<" "<<sy[1]<<" "<<st[1]<<endl;
    // cout<<"sxyt is "<<sx[2]<<" "<<sy[2]<<" "<<st[2]<<endl;
    ddouble_t px0 = -sx[0];
    ddouble_t py0;
    if (m_robotCtrl.supportStatus == LEFT_BASED) {
        py0 = -RobotPara::ankle_distance / 2 - RobotPara::yzmp;

    } else if (m_robotCtrl.supportStatus == RIGHT_BASED) {
        py0 = RobotPara::ankle_distance / 2 + RobotPara::yzmp;
        vxy[1] = -vxy[1];
    } else
        throw runtime_error("NEITHER LEFT_BASED NOR RIGHT_BASED");

    ///%%
    ddouble_t Tsup = RobotPara::stepnum * RobotPara::dt;
    ddouble_t a = 10, b = 1;
    ddouble_t Tc = sqrt(RobotPara::hipheight / RobotPara::g);
    ddouble_t C = cosh(Tsup / Tc);
    ddouble_t S = sinh(Tsup / Tc);
    int m = (int)sx.size();
    // ddouble_t px[m], py[m];
    vector<ddouble_t> px(m);
    vector<ddouble_t> py(m);
    /// foot pos

    ddouble_t tmp_a = 0;
    for (int i = 0; i < m; i++) {
        if (m_robotCtrl.supportStatus == LEFT_BASED)
            tmp_a = pow(-1, i + 1);
        else if (m_robotCtrl.supportStatus == RIGHT_BASED)
            tmp_a = -pow(-1, i + 1);

        if (i == 0) {
            px[i] = px0 + cos(st[i]) * sx[i] + sin(st[i]) * tmp_a * sy[i];
            py[i] = py0 - cos(st[i]) * tmp_a * sy[i] + sin(st[i]) * sx[i];
        } else {
            px[i] = px[i - 1] + cos(st[i]) * sx[i] + sin(st[i]) * tmp_a * sy[i];
            py[i] = py[i - 1] - cos(st[i]) * tmp_a * sy[i] + sin(st[i]) * sx[i];
        }
    }
    /// walk para
    vector<ddouble_t> cx(m);
    vector<ddouble_t> cy(m);
    for (int i = 0; i < m; i++) {
        if (m_robotCtrl.supportStatus == LEFT_BASED)
            tmp_a = pow(-1, i + 1);
        else if (m_robotCtrl.supportStatus == RIGHT_BASED)
            tmp_a = -pow(-1, i + 1);

        if (i == m - 1) {
            cx[i] = sx[i] / 2;
            cy[i] = tmp_a * sy[i] / 2;
        } else {
            cx[i] = sx[i + 1] / 2;
            cy[i] = tmp_a * sy[i + 1] / 2;
        }
        // cout<<cy[i]<<endl;
    }
    /// end speed
    // ddouble_t vx_e[m], vy_e[m];
    vector<ddouble_t> vx_e(m);
    vector<ddouble_t> vy_e(m);
    for (int n = 0; n < m; n++) {
        if (n == m - 1) {
            vx_e[n] = (C + 1) / Tc / S * cx[n];
            vy_e[n] = (C - 1) / Tc / S * cy[n];
        } else {
            vx_e[n] = (C + 1) / Tc / S * cx[n];
            vy_e[n] = (C - 1) / Tc / S * cy[n];
        }
        //   cout<<vy_e[n]<<endl;
    }
    ddouble_t D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);
    /// desired
    // ddouble_t x_d[m], vx_d[m], y_d[m], vy_d[m];

    vector<ddouble_t> x_d(m);
    vector<ddouble_t> vx_d(m);
    vector<ddouble_t> y_d(m);
    vector<ddouble_t> vy_d(m);
    for (int i = 0; i < m; i++) {
        if (i == m - 1) {
            x_d[i] = px[i] + cx[i] * cos(st[i]) - cy[i] * sin(st[i]);
            vx_d[i] = cos(st[i]) * vx_e[i] - sin(st[i]) * vy_e[i];
            y_d[i] = py[i] + cx[i] * sin(st[i]) + cy[i] * cos(st[i]);
            vy_d[i] = cos(st[i]) * vy_e[i] + sin(st[i]) * vx_e[i];

        } else {
            x_d[i] = px[i] + cx[i] * cos(st[i + 1]) - cy[i] * sin(st[i + 1]);
            vx_d[i] = cos(st[i + 1]) * vx_e[i] - sin(st[i + 1]) * vy_e[i];
            y_d[i] = py[i] + cx[i] * sin(st[i + 1]) + cy[i] * cos(st[i + 1]);
            vy_d[i] = cos(st[i + 1]) * vy_e[i] + sin(st[i + 1]) * vx_e[i];
        }
        // cout<<y_d[i]<<endl;
    }
    /// foot pos change_x
    // ddouble_t px_c[m];
    // ddouble_t x_f[m], vx_f[m];

    vector<ddouble_t> px_c(m);
    vector<ddouble_t> x_f(m);
    vector<ddouble_t> vx_f(m);

    for (int n = 0; n < m; n++) {
        if (n == 0) {
            px_c[n] = -a * (C - 1) / D * (x_d[n] - C * x_f0 - Tc * S * vxy[0]) - b * S / Tc / D * (vx_d[n] - S / Tc * x_f0 - C * vxy[0]);
            x_f[0] = C * x_f0 + Tc * S * vxy[0] + (1 - C) * px_c[0];
            vx_f[0] = S / Tc * x_f0 + C * vxy[0] - S / Tc * px_c[0];
        } else {
            px_c[n] = -a * (C - 1) / D * (x_d[n] - C * x_f[n - 1] - Tc * S * vx_f[n - 1]) - b * S / Tc / D * (vx_d[n] - S / Tc * x_f[n - 1] - C * vx_f[n - 1]);
            x_f[n] = C * x_f[n - 1] + Tc * S * vx_f[n - 1] + (1 - C) * px_c[n];
            vx_f[n] = S / Tc * x_f[n - 1] + C * vx_f[n - 1] - S / Tc * px_c[n];
        }
    }

    /// foot pos change_y
    // ddouble_t py_c[m];
    // ddouble_t y_f[m];
    // ddouble_t vy_f[m];

    vector<ddouble_t> py_c(m);
    vector<ddouble_t> y_f(m);
    vector<ddouble_t> vy_f(m);

    for (int n = 0; n < m; n++) {
        if (n == 0) {
            py_c[n] = -a * (C - 1) / D * (y_d[n] - C * y_f0 - Tc * S * vxy[1]) - b * S / Tc / D * (vy_d[n] - S / Tc * y_f0 - C * vxy[1]);
            y_f[0] = C * y_f0 + Tc * S * vxy[1] + (1 - C) * py_c[0];
            vy_f[0] = S / Tc * y_f0 + C * vxy[1] - S / Tc * py_c[0];
        } else {
            py_c[n] = -a * (C - 1) / D * (y_d[n] - C * y_f[n - 1] - Tc * S * vy_f[n - 1]) - b * S / Tc / D * (vy_d[n] - S / Tc * y_f[n - 1] - C * vy_f[n - 1]);
            y_f[n] = C * y_f[n - 1] + Tc * S * vy_f[n - 1] + (1 - C) * py_c[n];
            vy_f[n] = S / Tc * y_f[n - 1] + C * vy_f[n - 1] - S / Tc * py_c[n];
        }
        // cout<<py_c[n]<<endl;
    }

    /// HIP TRAJectory
    // ddouble_t* x_s = new ddouble_t[m_stepnum];
    // ddouble_t* y_s = new ddouble_t[m_stepnum];

    // ddouble_t xt[(m * RobotPara::stepnum)], yt[(m * RobotPara::stepnum)];
    vector<ddouble_t> xt(m * RobotPara::stepnum);
    vector<ddouble_t> yt(m * RobotPara::stepnum);
    int num = 0;
    for (int i = 0; i < m; i++) {
        if (i == 0) {
            for (ddouble_t t = RobotPara::dt; t < Tsup + RobotPara::dt; t = t + RobotPara::dt) {
                xt[num] = (x_f0 - px_c[i]) * cosh(t / Tc) + Tc * vxy[0] * sinh(t / Tc) + px_c[i];
                num++;
            }
        } else {
            for (ddouble_t t = RobotPara::dt; t < Tsup + RobotPara::dt; t = t + RobotPara::dt) {
                xt[num] = (x_f[i - 1] - px_c[i]) * cosh(t / Tc) + Tc * vx_f[i - 1] * sinh(t / Tc) + px_c[i];
                num++;
            }
        }
    }
    num = 0;
    for (int i = 0; i < m; i++) {
        if (i == 0) {
            for (ddouble_t t = RobotPara::dt; t < Tsup + RobotPara::dt; t = t + RobotPara::dt) {
                yt[num] = (y_f0 - py_c[i]) * cosh(t / Tc) + Tc * vxy[1] * sinh(t / Tc) + py_c[i];
                num++;
            }
        } else {
            for (ddouble_t t = RobotPara::dt; t < Tsup + RobotPara::dt; t = t + RobotPara::dt) {
                yt[num] = (y_f[i - 1] - py_c[i]) * cosh(t / Tc) + Tc * vy_f[i - 1] * sinh(t / Tc) + py_c[i];
                num++;
            }
        }
    }

    /// LEFTANDRIGHT ANKLE
    ddouble_t* r_s;
    ddouble_t* l_s;
    // vector<ddouble_t> r_s(RobotPara::stepnum);
    // vector<ddouble_t> l_s(RobotPara::stepnum);

    // ddouble_t leftAnkle[6],rightAnkle[6];
    RobotCtrl targetCtrl;
    /// set target (lifting leg)
    targetCtrl = m_robotCtrl;
    m_robotCtrl.setWalkMode();
    /*gait change leg x direction correct*/
    // cout<<left<<setw(10)<<"before and after"<<m_robotCtrl.robot_x<<tsx<<endl;
    ddouble_t x_amend;
    if (tsx >= m_robotCtrl.robot_x)
        x_amend = (-tsx + m_robotCtrl.robot_x) * RobotPara::x_compensation_acc;
    else
        x_amend = (-tsx + m_robotCtrl.robot_x) * RobotPara::x_compensation_dec;
    // cout<<x_amend<<endl;
    // cout<<m_robotCtrl.cm[0]<<endl;
    static ddouble_t cm_residul = 0; // important with x_amend
    if (m_robotCtrl.supportStatus == LEFT_BASED) {
        targetCtrl.ra[0] = px_c[1] + x_amend - cm_residul;
        // targetCtrl.ra[2] = 0;//fix
        targetCtrl.ra[5] = tst; // bug+ z_ra_y;//yaw
        targetCtrl.la[0] = px_c[0];
        if (tst >= 0)
            targetCtrl.ra[1] = py_c[1] + RobotPara::Ankle_dev_r + tst * abs(tsx) / RobotPara::mid_x_max * RobotPara::Ankle_dev_r_tk;
        else
            targetCtrl.ra[1] = py_c[1] + RobotPara::Ankle_dev_r;

        targetCtrl.la[1] = py_c[0] + RobotPara::Ankle_dev_l;
        targetCtrl.la[5] = m_robotCtrl.la[5]; // + z_ra_y              ;
        r_s = curveCreate(m_robotCtrl.ra[2], m_AnkleH_mid_r, m_AnkleH_last_r, RobotPara::stepnum);
        l_s = curveCreate(m_AnkleH_last_l, 0, 0, RobotPara::stepnum);

    } else {
        // } else if (m_robotCtrl.supportStatus == RIGHT_BASED) {
        targetCtrl.la[0] = px_c[1] + x_amend - cm_residul;
        if (tst <= 0)
            targetCtrl.la[1] = py_c[1] + RobotPara::Ankle_dev_l - tst * abs(tsx) / RobotPara::mid_x_max * RobotPara::Ankle_dev_l_tk;
        else
            targetCtrl.la[1] = py_c[1] + RobotPara::Ankle_dev_l;
        targetCtrl.la[5] = tst; //+ z_la_y;
        targetCtrl.ra[0] = px_c[0];
        targetCtrl.ra[1] = py_c[0] + RobotPara::Ankle_dev_r;
        targetCtrl.ra[5] = m_robotCtrl.ra[5];
        l_s = curveCreate(m_robotCtrl.la[2], m_AnkleH_mid_l, m_AnkleH_last_l, RobotPara::stepnum);
        r_s = curveCreate(m_AnkleH_last_r, 0, 0, RobotPara::stepnum);
    }

    targetCtrl.cm[0] = 0;
    targetCtrl.cm[1] = 0;
    targetCtrl.cm[2] = RobotPara::hipheight;
    targetCtrl.cm[3] = RobotPara::cm_r;
    targetCtrl.cm[4] = RobotPara::cm_p;
    targetCtrl.cm[5] = RobotPara::cm_y + tst;
    // targetCtrl.cm[3] = m_robotCtrl.cm[3];
    // targetCtrl.cm[4] = m_robotCtrl.cm[4];
    // targetCtrl.cm[5] = m_robotCtrl.cm[5] + tst;
    targetCtrl.rh[0] = RobotPara::arm_crouch_p;
    targetCtrl.rh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.lh[0] = RobotPara::arm_crouch_p;
    targetCtrl.lh[1] = RobotPara::arm_crouch_theta;

    /*param k in walking */
    if (rsy >= 0) // leftmove
    {
        targetCtrl.la[3] = RobotPara::la_r + RobotPara::la_dr_lk * fabs(rsy);
        targetCtrl.ra[3] = RobotPara::ra_r + RobotPara::ra_dr_lk * fabs(rsy);
        targetCtrl.cm_dxy[1] = RobotPara::cm_dy + RobotPara::cm_dy_lk * fabs(rsy);
    } else // rightmove
    {
        targetCtrl.la[3] = RobotPara::la_r + RobotPara::la_dr_rk * fabs(rsy);
        targetCtrl.ra[3] = RobotPara::ra_r + RobotPara::ra_dr_rk * fabs(rsy);
        targetCtrl.cm_dxy[1] = RobotPara::cm_dy + RobotPara::cm_dy_rk * fabs(rsy);
    }
    if (rsx >= 0) {
        targetCtrl.cm_dxy[0] = RobotPara::cm_dx + RobotPara::cm_dx_fk * rsx;
        targetCtrl.la[4] = RobotPara::la_p + RobotPara::la_dp_fk * rsx;
        targetCtrl.ra[4] = RobotPara::ra_p + RobotPara::ra_dp_fk * rsx;
        targetCtrl.cm[4] = RobotPara::cm_p + RobotPara::cm_dp_fk * rsx;

    } else {
        targetCtrl.cm_dxy[0] = RobotPara::cm_dx - RobotPara::cm_dx_bk * rsx;
        targetCtrl.la[4] = RobotPara::la_p - RobotPara::la_dp_bk * rsx;
        targetCtrl.ra[4] = RobotPara::ra_p - RobotPara::ra_dp_bk * rsx;
    }

    /* GET ANGLE */
    int dataArray[MOTORNUM];
    for (int i = 0; i < RobotPara::stepnum; i++) {
        targetCtrl.ra[2] = r_s[i];
        targetCtrl.la[2] = l_s[i];
        targetCtrl.cm[0] = xt[i] - cm_residul;
        targetCtrl.cm[1] = yt[i];
        // ddouble_t currentAngleZ = (m_status->getAngledata()).angleZ;
        m_robotCtrl.supportNum = i + 1;
        m_robotCtrl.num_left = RobotPara::stepnum - i;
        // m_status->updateDeltaDist(VecPos(sx[1]*1.0*m_stepK  / m_stepnum, tsy /
        // m_stepnum), tst / m_stepnum);

        ddouble_t dx, dy, dt;

        dx = rsx * 1.0 * RobotPara::stepK / RobotPara::stepnum;
        dx *= dx > 0 ? RobotPara::forward_k : RobotPara::back_k;
        dy = rsy / RobotPara::stepnum;
        dy *= dy > 0 ? RobotPara::left_k : RobotPara::right_k;
        dt = rst / RobotPara::stepnum * RobotPara::angle_k;

        m_status->updateDeltaDist(dx, dy, dt);
        getAngle_serial(targetCtrl, dataArray, 1);
        doTxTask(dataArray);
    }
    if (m_robotCtrl.supportStatus == LEFT_BASED) {
        m_robotCtrl.supportStatus = RIGHT_BASED;
    } else if (m_robotCtrl.supportStatus == RIGHT_BASED) {
        m_robotCtrl.supportStatus = LEFT_BASED;
    } else {
        ROS_ERROR("HumanRobot::Leg_up robotCtrl.supportStatus not right or left");
    }
    cm_residul = x_amend;
}

ddouble_t
HumanRobot::getThetaAmend(const ddouble_t tsx)
{
    ddouble_t k, theta_amd;
    if (tsx <= 0) {
        k = (RobotPara::step_theta_amend - RobotPara::back_theta_amend) / (0 - RobotPara::back_x_max);
        theta_amd = RobotPara::back_theta_amend + k * (tsx - RobotPara::back_x_max);
    } else if (tsx <= RobotPara::mid_x_max) {
        k = (RobotPara::mid_theta_amend - RobotPara::step_theta_amend) / (RobotPara::mid_x_max - 0);
        theta_amd = RobotPara::step_theta_amend + k * (tsx - 0);
    } else {
        if (abs(RobotPara::top_x_max - RobotPara::mid_x_max) < epso)
            k = 0;
        else
            k = (RobotPara::top_theta_amend - RobotPara::mid_theta_amend) / (RobotPara::top_x_max - RobotPara::mid_x_max);
        theta_amd = RobotPara::mid_theta_amend + k * (tsx - RobotPara::mid_x_max);
    }

    return theta_amd;
}

ddouble_t
HumanRobot::getRightAnkleMidHigh(const ddouble_t tsx)
{
    ddouble_t k, ah;
    if (abs(tsx) <= RobotPara::mid_x_max) {
        k = (RobotPara::ah_mr_mid - RobotPara::ah_mr_zero) / (RobotPara::mid_x_max - 0);
        ah = RobotPara::ah_mr_zero + k * (tsx - 0);
    } else {
        if (abs(RobotPara::top_x_max - RobotPara::mid_x_max) < epso)
            k = 0;
        else
            k = (RobotPara::ah_mr_mid - RobotPara::ah_mr_zero) / (RobotPara::top_x_max - RobotPara::mid_x_max);
        ah = RobotPara::ah_mr_mid + k * (tsx - RobotPara::mid_x_max);
    }
    return ah;
}

ddouble_t
HumanRobot::getLeftAnkleMidHigh(const ddouble_t tsx)
{
    ddouble_t k, ah;
    if (abs(tsx) <= RobotPara::mid_x_max) {
        k = (RobotPara::ah_ml_mid - RobotPara::ah_ml_zero) / (RobotPara::mid_x_max - 0);
        ah = RobotPara::ah_ml_zero + k * (tsx - 0);
    } else {
        if (abs(RobotPara::top_x_max - RobotPara::mid_x_max) < epso)
            k = 0;
        else
            k = (RobotPara::ah_ml_top - RobotPara::ah_ml_mid) / (RobotPara::top_x_max - RobotPara::mid_x_max);
        ah = RobotPara::ah_ml_mid + k * (tsx - RobotPara::mid_x_max);
    }
    return ah;
}

/************************************************
 * \brief getVxyf0
 * get robot vx according to the robot foot length
 * \param tsx the robot's steplength sx
 *
 ***********************************************/
void
HumanRobot::getVxyf0(ddouble_t tsx, ddouble_t vxy[])
{
    vector<ddouble_t> sx, sy, st;
    sx.push_back(m_robotCtrl.robot_x);
    sy.push_back(RobotPara::ankle_distance + RobotPara::yzmp + RobotPara::yzmp);
    st.push_back(0);

    ddouble_t px0 = -m_robotCtrl.robot_x;
    ddouble_t py0 = -RobotPara::ankle_distance / 2 - RobotPara::yzmp;

    ///%%
    ddouble_t Tsup = RobotPara::stepnum * RobotPara::dt;
    // ddouble_t a=10,b=1;
    ddouble_t Tc = sqrt(RobotPara::hipheight / RobotPara::g);
    ddouble_t C = cosh(Tsup / Tc);
    ddouble_t S = sinh(Tsup / Tc);
    int m = 1;
    // ddouble_t px[m], py[m];
    vector<ddouble_t> px(m);
    vector<ddouble_t> py(m);

    /// foot pos
    for (int i = 0; i < m; i++) {
        if (i == 0) {
            px[i] = px0 + cos(st[i]) * sx[i] + sin(st[i]) * pow(-1, i + 1) * sy[i];
            py[i] = py0 - cos(st[i]) * pow(-1, i + 1) * sy[i] + sin(st[i]) * sx[i];
        } else {
            px[i] = px[i - 1] + cos(st[i]) * sx[i] + sin(st[i]) * pow(-1, i + 1) * sy[i];
            py[i] = py[i - 1] - cos(st[i]) * pow(-1, i + 1) * sy[i] + sin(st[i]) * sx[i];
        }
    }
    /// walk para
    // ddouble_t cx[m], cy[m];
    vector<ddouble_t> cx(m);
    vector<ddouble_t> cy(m);
    for (int i = 0; i < m; i++) {
        if (i == m - 1) {
            cx[i] = sx[i] / 2;
            cy[i] = pow(-1, i + 1) * sy[i] / 2;
        } else {
            cx[i] = sx[i + 1] / 2;
            cy[i] = pow(-1, i + 1) * sy[i + 1] / 2;
        }
        // cout<<cy[i]<<endl;
    }
    /// end speed
    // ddouble_t vx_e[m], vy_e[m];
    vector<ddouble_t> vx_e(m);
    vector<ddouble_t> vy_e(m);
    for (int n = 0; n < m; n++) {
        if (n == m - 1) {
            vx_e[n] = (C + 1) / Tc / S * cx[n];
            vy_e[n] = (C - 1) / Tc / S * cy[n];
        } else {
            vx_e[n] = (C + 1) / Tc / S * cx[n];
            vy_e[n] = (C - 1) / Tc / S * cy[n];
        }
        //   cout<<vy_e[n]<<endl;
    }
    // ddouble_t D=a*(C-1)*(C-1)+b*(S/Tc)*(S/Tc);
    /// desired
    // ddouble_t x_d[m], y_d[m];
    // ddouble_t vy_d[m], vx_d[m];

    vector<ddouble_t> vy_d(m);
    vector<ddouble_t> vx_d(m);
    for (int i = 0; i < m; i++) {
        if (i == m - 1) {
            //      x_d[i] = px[i] + cx[i] * cos(st[i]) - cy[i] * sin(st[i]);
            vx_d[i] = cos(st[i]) * vx_e[i] - sin(st[i]) * vy_e[i];
            //      y_d[i] = py[i] + cx[i] * sin(st[i]) + cy[i] * cos(st[i]);
            vy_d[i] = cos(st[i]) * vy_e[i] + sin(st[i]) * vx_e[i];

        } else {
            //      x_d[i] = px[i] + cx[i] * cos(st[i + 1]) - cy[i] * sin(st[i + 1]);
            vx_d[i] = cos(st[i + 1]) * vx_e[i] - sin(st[i + 1]) * vy_e[i];
            //      y_d[i] = py[i] + cx[i] * sin(st[i + 1]) + cy[i] * cos(st[i + 1]);
            vy_d[i] = cos(st[i + 1]) * vy_e[i] + sin(st[i + 1]) * vx_e[i];
        }
    }
    vxy[0] = vx_d[0];
    vxy[1] = abs(vy_d[0]);
}

/************************************************
 * \brief runWalk
 * the interface function in robot walking
 * notice: only one step
 * \param tsx the steplength of the robot x direction
 * \param tsy the width of the robot y direction
 * \param tst the rotation angle of the robot in only one step
 * \return null
p ***********************************************/
void
HumanRobot::runWalk(ddouble_t tsx, ddouble_t tsy, ddouble_t tst)
{
    // cout << tsy << endl;
    ddouble_t r_sx = tsx; // real in world
    ddouble_t r_sy = tsy;
    ddouble_t r_st = tst;
    // ddouble_t c_sx , c_sy , c_st; // command in robot
    if (true) { // do smooth in real , can also do in command
        RobotCtrl tmp_xyt(m_robotCtrl);
        // cout <<"start" << endl;
        // cout << m_robotCtrl.getWalkVelY() << endl;
        if (tsy - tmp_xyt.robot_y >= 1.0f) // smooth in y direction
            r_sy = tmp_xyt.robot_y + 1.0f;
        if (tsy - tmp_xyt.robot_y <= -1.0f)
            r_sy = tmp_xyt.robot_y - 1.0f;

        if (abs(tsx - tmp_xyt.robot_x) >= 2.5f && *m_manager->gaitState != WALKRIGHTKICK && *m_manager->gaitState != WALKLEFTKICK) {
            if (tsx >= tmp_xyt.robot_x) // acc
                r_sx = min(tmp_xyt.robot_x + 2.5f, tsx); //(tsx+tmp_xyt.robot_x)/2;
            else
                r_sx = max(tmp_xyt.robot_x - 2.5f, tsx);
        }
    }

    // ddouble_t lst_amd = getThetaAmend(tsx);
    // c_st = r_st + lst_amd;
    // c_sx = r_sx + RobotPara::step_x_amend;
    // c_sy = r_sy;
    m_AnkleH_mid_l = getLeftAnkleMidHigh(tsx);
    m_AnkleH_mid_r = getRightAnkleMidHigh(tsx);
    // notice: motorconfig.cfg in simulation and in real robot must be the same
    Leg_up(r_sx, r_sy, r_st);

    // lsy ,lst - lst_amd);

    m_robotCtrl.robot_x = r_sx;
    m_robotCtrl.robot_y = r_sy;
    //     sy[1] - RobotPara::yzmp - RobotPara::yzmp - RobotPara::ankle_distance;
    m_robotCtrl.robot_t = r_st;
}

/************************************************
 * \brief getTheta
 * angle limit function
 * \param v the robot speed (sx)
 * \return angle limit
 *
 ***********************************************/
ddouble_t
HumanRobot::getTheta(ddouble_t v)
{
    if (v >= 4)
        return 5;
    else if (v < 4 && v > 2)
        return 10;
    else if (v <= 2 && v >= -2)
        return 15;
    else
        return 10;
}

/************************************************
 * \brief getAngle_serial
 * inverse kinetics of the parallel's robot .unit:cm,angle
 * \param ra_target rightAnkle[6] right Ankle's x y z r p y
 * \param la_target leftAnkle[6] left Ankle's x y z r p y
 * \param hipA hipA[6] the x y z r p y of the center of mass
 * \param dataArray[20] return data
 * \return null
 *
 ***********************************************/
void
HumanRobot::getAngle_serial(const RobotCtrl targetCtrl, int dataArray[], const bool isExcute)
{
    ddouble_t hip_target[6];
    for (int i = 0; i < 6; i++) {
        hip_target[i] = targetCtrl.cm[i];
    }
    if (isExcute) // && m_robotCtrl.supportStatus != DOUBLE_BASED)//bug
    {
        hip_target[3] = hip_target[3]; // + m_cm_r;
        hip_target[4] = hip_target[4]; // + m_cm_p;
        hip_target[5] = hip_target[5]; // + m_cm_y;
    }

    ///======================================================

    ddouble_t cm[6], ra[6], la[6], rh[6], lh[6], H_cm[6], H_ra[6], H_la[6], H_rh[6], H_lh[6];
    // onedCtrl 为
    // 一维轨迹的生成类，用来规划摆动腿的生成轨迹，输入vmax和amax分别代表了最大速度和最大加速度。
    //原理详情可以参考《全方位移动机器人运动控制及规划》.吴永海. 4.3 1D轨迹规划
    //这里取值很大，代表完全跟踪上的意思。
    onedCtrl ankleCtrl = onedCtrl(9000, 50000); // for ra[2],la[2] xy direction
    onedCtrl cmdxyCtrl = onedCtrl(4000, 20000); // for cm_dxy[0-1]
    onedCtrl rpyCtrl = onedCtrl(2000, 10000); // for cm_rpy,cm[3-5]

    for (int i = 0; i < 6; i++) {
        ra[i] = targetCtrl.ra[i];
        la[i] = targetCtrl.la[i];
        cm[i] = hip_target[i];
        rh[i] = targetCtrl.rh[i];
        lh[i] = targetCtrl.lh[i];
        H_ra[i] = targetCtrl.ra[i];
        H_la[i] = targetCtrl.la[i];
        H_cm[i] = hip_target[i];
        H_rh[i] = targetCtrl.rh[i];
        H_lh[i] = targetCtrl.lh[i];
    }

    /// we are not concerned:
    /// ra[2],la[2] z dirction( spline template)
    /// cm[0-1] (x,y's direction of center of mass),

    /// we are concerned
    /// ra[0-1],la[0-1](xy direction)
    /// ra[6],la[6](yal direction)
    /// cm[2-5](cmz,cm_roll,pitch,yal)
    /// cm_dxy[0-1](cm_dx,cm_dy offset):last deal

    if (isExcute) {
        ///====================leftHand=======================================
        /// lh[0] left ankle p's direction
        ddouble_t lhx_start[2], lhx_target[2], lhx_out[2];
        ddouble_t lhy_start[2], lhy_target[2], lhy_out[2];
        ddouble_t lhz_start[2], lhz_target[2], lhz_out[2];
        ddouble_t lhroll_start[2], lhroll_target[2], lhroll_out[2];
        ddouble_t lhpitch_start[2], lhpitch_target[2], lhpitch_out[2];
        ddouble_t lhyaw_start[2], lhyaw_target[2], lhyaw_out[2];
        lhx_start[0] = m_robotCtrl.lh[0];
        lhx_start[1] = m_robotCtrl.lh_v[0]; // m_robotCtrl.lh_v[0];
        lhx_target[0] = targetCtrl.lh[0];
        lhx_target[1] = 0;
        ankleCtrl.oned_analysis(lhx_out, lhx_start, lhx_target, m_robotCtrl.num_left);
        /// lh[1] left ankle theta's direction
        lhy_start[0] = m_robotCtrl.lh[1];
        lhy_start[1] = m_robotCtrl.lh_v[1];
        lhy_target[0] = targetCtrl.lh[1];
        lhy_target[1] = 0;
        rpyCtrl.oned_analysis(lhy_out, lhy_start, lhy_target, m_robotCtrl.num_left);
        /// lh[2] left ankle z's direction especially for DOUBLE_BASED
        lhz_start[0] = m_robotCtrl.lh[2];
        lhz_start[1] = m_robotCtrl.lh_v[2];
        lhz_target[0] = targetCtrl.lh[2];
        lhz_target[1] = 0;
        ankleCtrl.oned_analysis(lhz_out, lhz_start, lhz_target, m_robotCtrl.num_left);
        /// lh[3] left ankle roll's direction
        lhroll_start[0] = m_robotCtrl.lh[3];
        lhroll_start[1] = m_robotCtrl.lh_v[3];
        lhroll_target[0] = targetCtrl.lh[3];
        lhroll_target[1] = 0;
        rpyCtrl.oned_analysis(lhroll_out, lhroll_start, lhroll_target, m_robotCtrl.num_left);
        /// lh[4] left ankle pitch's direction
        lhpitch_start[0] = m_robotCtrl.lh[4];
        lhpitch_start[1] = m_robotCtrl.lh_v[4];
        lhpitch_target[0] = targetCtrl.lh[4];
        lhpitch_target[1] = 0;
        rpyCtrl.oned_analysis(lhpitch_out, lhpitch_start, lhpitch_target, m_robotCtrl.num_left);
        /// lh[5] left ankle yaw's direction
        lhyaw_start[0] = m_robotCtrl.lh[5];
        lhyaw_start[1] = m_robotCtrl.lh_v[5];
        lhyaw_target[0] = targetCtrl.lh[5];
        lhyaw_target[1] = 0;
        rpyCtrl.oned_analysis(lhyaw_out, lhyaw_start, lhyaw_target, m_robotCtrl.num_left);
        /// lh update
        lh[0] = lhx_out[0];
        lh[1] = lhy_out[0];
        lh[2] = lhz_out[0];
        lh[3] = lhroll_out[0]; // to be discussed
        lh[4] = lhpitch_out[0]; // to be discussed
        lh[5] = lhyaw_out[0];
        /// m_robotCtrl update lh[0-5] lh_v[0-5]
        for (int i = 0; i < 6; i++) {
            m_robotCtrl.lh[i] = lh[i];
            m_robotCtrl.lh_v[i] = 0;
        }
        m_robotCtrl.lh_v[0] = lhx_out[1];
        m_robotCtrl.lh_v[1] = lhy_out[1];
        m_robotCtrl.lh_v[2] = lhz_out[1];
        m_robotCtrl.lh_v[3] = lhroll_out[1];
        m_robotCtrl.lh_v[4] = lhpitch_out[1];
        m_robotCtrl.lh_v[5] = lhyaw_out[1];

        ///===================rightHand===================================
        /// ra[0] right hand p's direction
        ddouble_t rhx_start[2], rhx_target[2], rhx_out[2];
        ddouble_t rhy_start[2], rhy_target[2], rhy_out[2];
        ddouble_t rhz_start[2], rhz_target[2], rhz_out[2];
        ddouble_t rhroll_start[2], rhroll_target[2], rhroll_out[2];
        ddouble_t rhpitch_start[2], rhpitch_target[2], rhpitch_out[2];
        ddouble_t rhyaw_start[2], rhyaw_target[2], rhyaw_out[2];
        rhx_start[0] = m_robotCtrl.rh[0];
        rhx_start[1] = m_robotCtrl.rh_v[0]; // m_robotCtrl.rh_v[0];
        rhx_target[0] = targetCtrl.rh[0];
        rhx_target[1] = 0;
        ankleCtrl.oned_analysis(rhx_out, rhx_start, rhx_target, m_robotCtrl.num_left);
        /// rh[1] right hand y's direction
        rhy_start[0] = m_robotCtrl.rh[1];
        rhy_start[1] = m_robotCtrl.rh_v[1];
        rhy_target[0] = targetCtrl.rh[1];
        rhy_target[1] = 0;
        rpyCtrl.oned_analysis(rhy_out, rhy_start, rhy_target, m_robotCtrl.num_left);
        /// rh[2] right hand z's direction especially for DOUBLE_BASED
        rhz_start[0] = m_robotCtrl.rh[2];
        rhz_start[1] = m_robotCtrl.rh_v[2];
        rhz_target[0] = targetCtrl.rh[2];
        rhz_target[1] = 0;
        ankleCtrl.oned_analysis(rhz_out, rhz_start, rhz_target, m_robotCtrl.num_left);
        /// rh[3] right hand roll's direction
        rhroll_start[0] = m_robotCtrl.rh[3];
        rhroll_start[1] = m_robotCtrl.rh_v[3];
        rhroll_target[0] = targetCtrl.rh[3];
        rhroll_target[1] = 0;
        rpyCtrl.oned_analysis(rhroll_out, rhroll_start, rhroll_target, m_robotCtrl.num_left);
        /// rh[4] right hand pitch's direction
        rhpitch_start[0] = m_robotCtrl.rh[4];
        rhpitch_start[1] = m_robotCtrl.rh_v[4];
        rhpitch_target[0] = targetCtrl.rh[4];
        rhpitch_target[1] = 0;
        rpyCtrl.oned_analysis(rhpitch_out, rhpitch_start, rhpitch_target, m_robotCtrl.num_left);
        /// rh[5] right hand yal's direction
        rhyaw_start[0] = m_robotCtrl.rh[5];
        rhyaw_start[1] = m_robotCtrl.rh_v[5];
        rhyaw_target[0] = targetCtrl.rh[5];
        rhyaw_target[1] = 0;
        rpyCtrl.oned_analysis(rhyaw_out, rhyaw_start, rhyaw_target, m_robotCtrl.num_left);
        /// rh update
        rh[0] = rhx_out[0];
        rh[1] = rhy_out[0];
        rh[2] = rhz_out[0]; // spline,to be discussed
        rh[3] = rhroll_out[0]; // to be discussed
        rh[4] = rhpitch_out[0]; // to be discussed
        rh[5] = rhyaw_out[0];
        /// m_robotCtrl update rh[0-5] rh_v[0-5]
        for (int i = 0; i < 6; i++) {
            m_robotCtrl.rh[i] = rh[i];
            m_robotCtrl.rh_v[i] = 0;
        }
        m_robotCtrl.rh_v[0] = rhx_out[1];
        m_robotCtrl.rh_v[1] = rhy_out[1];
        m_robotCtrl.rh_v[2] = rhz_out[1]; // DOUBLE_BASED
        m_robotCtrl.rh_v[3] = rhroll_out[1];
        m_robotCtrl.rh_v[4] = rhpitch_out[1];
        m_robotCtrl.rh_v[5] = rhyaw_out[1];
        ///====================leftAnkle=======================================
        /// la[0] left ankle x's direction
        ddouble_t lax_start[2], lax_target[2], lax_out[2];
        ddouble_t lay_start[2], lay_target[2], lay_out[2];
        ddouble_t laz_start[2], laz_target[2], laz_out[2];
        ddouble_t laroll_start[2], laroll_target[2], laroll_out[2];
        ddouble_t lapitch_start[2], lapitch_target[2], lapitch_out[2];
        ddouble_t layaw_start[2], layaw_target[2], layaw_out[2];
        lax_start[0] = m_robotCtrl.la[0];
        lax_start[1] = m_robotCtrl.la_v[0]; // m_robotCtrl.la_v[0];
        lax_target[0] = targetCtrl.la[0];
        lax_target[1] = 0;
        if (m_leftkick_flag) // kick
        {
            onedCtrl tmpCtrl = onedCtrl(90000, 500000); // for cm_rpy,cm[3-5]
            int tmp_num_left;
            int tmp_10;
            tmp_10 = (int)((1 - RobotPara::kickPercent) * RobotPara::stepnum);
            tmp_num_left = m_robotCtrl.num_left - tmp_10; // abs
            if (tmp_num_left <= 0)
                tmp_num_left = 0;
            tmpCtrl.oned_analysis(lax_out, lax_start, lax_target, tmp_num_left);
        } else if (lax_target[0] < -1.5) // backmove
        {
            int num_left_lax;
            int tmp_10_lax;
            tmp_10_lax = (int)((1 - RobotPara::percent_bx) * RobotPara::stepnum);
            num_left_lax = m_robotCtrl.num_left - tmp_10_lax; // abs
            if (num_left_lax <= 0)
                num_left_lax = 0;
            ankleCtrl.oned_analysis(lax_out, lax_start, lax_target, num_left_lax);
        } else { // walk
            int num_left_lax;
            int tmp_10_lax;
            tmp_10_lax = (int)((1 - RobotPara::percent_fx) * RobotPara::stepnum);
            num_left_lax = m_robotCtrl.num_left - tmp_10_lax; // abs
            if (num_left_lax <= 0)
                num_left_lax = 0;
            ankleCtrl.oned_analysis(lax_out, lax_start, lax_target, num_left_lax);
        }
        /// la[1] left ankle y's direction
        lay_start[0] = m_robotCtrl.la[1];
        lay_start[1] = m_robotCtrl.la_v[1];
        lay_target[0] = targetCtrl.la[1];
        lay_target[1] = 0;
        int num_left_lay;
        int tmp_10_lay;
        tmp_10_lay = (int)((1 - RobotPara::percent_ly) * RobotPara::stepnum);
        num_left_lay = m_robotCtrl.num_left - tmp_10_lay; // abs
        if (num_left_lay <= 0)
            num_left_lay = 0;
        ankleCtrl.oned_analysis(lay_out, lay_start, lay_target, num_left_lay);
        // ankleCtrl.oned_analysis(lay_out,lay_start,lay_target,m_robotCtrl.num_left);
        /// la[2] left ankle z's direction especially for DOUBLE_BASED
        laz_start[0] = m_robotCtrl.la[2];
        laz_start[1] = m_robotCtrl.la_v[2];
        laz_target[0] = targetCtrl.la[2];
        laz_target[1] = 0;
        ankleCtrl.oned_analysis(laz_out, laz_start, laz_target, m_robotCtrl.num_left);
        /// la[3] left ankle roll's direction
        laroll_start[0] = m_robotCtrl.la[3];
        laroll_start[1] = m_robotCtrl.la_v[3];
        laroll_target[0] = targetCtrl.la[3];
        laroll_target[1] = 0;
        rpyCtrl.oned_analysis(laroll_out, laroll_start, laroll_target, m_robotCtrl.num_left);
        /// la[4] left ankle pitch's direction
        lapitch_start[0] = m_robotCtrl.la[4];
        lapitch_start[1] = m_robotCtrl.la_v[4];
        lapitch_target[0] = targetCtrl.la[4];
        lapitch_target[1] = 0;
        rpyCtrl.oned_analysis(lapitch_out, lapitch_start, lapitch_target, m_robotCtrl.num_left);
        /// la[5] left ankle yaw's direction
        layaw_start[0] = m_robotCtrl.la[5];
        layaw_start[1] = m_robotCtrl.la_v[5];
        layaw_target[0] = targetCtrl.la[5];
        layaw_target[1] = 0;
        rpyCtrl.oned_analysis(layaw_out, layaw_start, layaw_target, m_robotCtrl.num_left);
        /// la update
        if (m_robotCtrl.auto_la[0]) {
            la[0] = lax_out[0];
            m_robotCtrl.la_v[0] = lax_out[1];
        } else {
            la[0] = targetCtrl.la[0];
            m_robotCtrl.la_v[0] = 0;
        }
        if (m_robotCtrl.auto_la[1]) {
            la[1] = lay_out[0];
            m_robotCtrl.la_v[1] = lay_out[1];
        } else {
            la[1] = targetCtrl.la[1];
            m_robotCtrl.la_v[1] = 0;
        }
        if (m_robotCtrl.auto_la[2]) {
            la[2] = laz_out[0];
            m_robotCtrl.la_v[2] = laz_out[1];
        } else {
            la[2] = targetCtrl.la[2];
            m_robotCtrl.la_v[2] = 0;
        }
        la[3] = laroll_out[0]; // to be discussed
        la[4] = lapitch_out[0]; // to be discussed
        la[5] = layaw_out[0];
        /// m_robotCtrl update la[0-5] la_v[0-5]
        for (int i = 0; i < 6; i++) {
            m_robotCtrl.la[i] = la[i];
        }
        m_robotCtrl.la_v[3] = laroll_out[1];
        m_robotCtrl.la_v[4] = lapitch_out[1];
        m_robotCtrl.la_v[5] = layaw_out[1];

        ///===================rightAnkle===================================
        /// ra[0] right ankle x's direction
        ddouble_t rax_start[2], rax_target[2], rax_out[2];
        ddouble_t ray_start[2], ray_target[2], ray_out[2];
        ddouble_t raz_start[2], raz_target[2], raz_out[2];
        ddouble_t raroll_start[2], raroll_target[2], raroll_out[2];
        ddouble_t rapitch_start[2], rapitch_target[2], rapitch_out[2];
        ddouble_t rayaw_start[2], rayaw_target[2], rayaw_out[2];
        rax_start[0] = m_robotCtrl.ra[0];
        rax_start[1] = m_robotCtrl.ra_v[0]; // m_robotCtrl.ra_v[0];
        rax_target[0] = targetCtrl.ra[0];
        rax_target[1] = 0;
        if (m_rightkick_flag) {
            onedCtrl tmpCtrl = onedCtrl(90000, 500000); // for cm_rpy,cm[3-5]
            int tmp_num_left;
            int tmp_10;
            tmp_10 = (int)((1 - RobotPara::kickPercent) * RobotPara::stepnum);
            tmp_num_left = m_robotCtrl.num_left - tmp_10; // abs
            if (tmp_num_left <= 0)
                tmp_num_left = 0;
            tmpCtrl.oned_analysis(rax_out, rax_start, rax_target, tmp_num_left);
        } else if (rax_target[0] < -1.5) // backmove
        {
            int num_left_rax;
            int tmp_10_rax;
            tmp_10_rax = (int)((1 - RobotPara::percent_bx) * RobotPara::stepnum);
            num_left_rax = m_robotCtrl.num_left - tmp_10_rax; // abs
            if (num_left_rax <= 0)
                num_left_rax = 0;
            ankleCtrl.oned_analysis(rax_out, rax_start, rax_target, num_left_rax);
        } else { // walk
            int num_left_rax;
            int tmp_10_rax;
            tmp_10_rax = (int)((1 - RobotPara::percent_fx) * RobotPara::stepnum);
            num_left_rax = m_robotCtrl.num_left - tmp_10_rax; // abs
            if (num_left_rax <= 0)
                num_left_rax = 0;
            ankleCtrl.oned_analysis(rax_out, rax_start, rax_target, num_left_rax);
        }

        /// ra[1] right ankle y's direction
        ray_start[0] = m_robotCtrl.ra[1];
        ray_start[1] = m_robotCtrl.ra_v[1];
        ray_target[0] = targetCtrl.ra[1];
        ray_target[1] = 0;
        int tmp_num_leftr; // template feature
        int tmp_10r;
        tmp_10r = (int)((1 - RobotPara::percent_ry) * RobotPara::stepnum);
        tmp_num_leftr = m_robotCtrl.num_left - tmp_10r; // abs
        if (tmp_num_leftr <= 0)
            tmp_num_leftr = 0;
        ankleCtrl.oned_analysis(ray_out, ray_start, ray_target, tmp_num_leftr);
        // ankleCtrl.oned_analysis(ray_out,ray_start,ray_target,m_robotCtrl.num_left);
        /// ra[2] right ankle z's direction especially for DOUBLE_BASED
        raz_start[0] = m_robotCtrl.ra[2];
        raz_start[1] = m_robotCtrl.ra_v[2];
        raz_target[0] = targetCtrl.ra[2];
        raz_target[1] = 0;
        ankleCtrl.oned_analysis(raz_out, raz_start, raz_target, m_robotCtrl.num_left);
        /// ra[3] left ankle roll's direction
        raroll_start[0] = m_robotCtrl.ra[3];
        raroll_start[1] = m_robotCtrl.ra_v[3];
        raroll_target[0] = targetCtrl.ra[3];
        raroll_target[1] = 0;
        rpyCtrl.oned_analysis(raroll_out, raroll_start, raroll_target, m_robotCtrl.num_left);
        /// ra[4] left ankle pitch's direction
        rapitch_start[0] = m_robotCtrl.ra[4];
        rapitch_start[1] = m_robotCtrl.ra_v[4];
        rapitch_target[0] = targetCtrl.ra[4];
        rapitch_target[1] = 0;
        rpyCtrl.oned_analysis(rapitch_out, rapitch_start, rapitch_target, m_robotCtrl.num_left);
        /// ra[5] right ankle yal's direction
        rayaw_start[0] = m_robotCtrl.ra[5];
        rayaw_start[1] = m_robotCtrl.ra_v[5];
        rayaw_target[0] = targetCtrl.ra[5];
        rayaw_target[1] = 0;
        rpyCtrl.oned_analysis(rayaw_out, rayaw_start, rayaw_target, m_robotCtrl.num_left);
        /// ra update
        if (m_robotCtrl.auto_ra[0]) {
            ra[0] = rax_out[0];
            m_robotCtrl.ra_v[0] = rax_out[1];
        } else {
            ra[0] = targetCtrl.ra[0];
            m_robotCtrl.ra_v[0] = 0;
        }
        if (m_robotCtrl.auto_ra[1]) {
            ra[1] = ray_out[0];
            m_robotCtrl.ra_v[1] = ray_out[1];
        } else {
            ra[1] = targetCtrl.ra[1];
            m_robotCtrl.ra_v[1] = 0;
        }
        if (m_robotCtrl.auto_ra[2]) {
            ra[2] = raz_out[0];
            m_robotCtrl.ra_v[2] = raz_out[1];
        } else {
            ra[2] = targetCtrl.ra[2];
            m_robotCtrl.ra_v[2] = 0;
        }
        ra[3] = raroll_out[0]; // to be discussed
        ra[4] = rapitch_out[0]; // to be discussed
        ra[5] = rayaw_out[0];
        /// m_robotCtrl update ra[0-5] ra_v[0-5]
        for (int i = 0; i < 6; i++) {
            m_robotCtrl.ra[i] = ra[i];
        }
        m_robotCtrl.ra_v[3] = raroll_out[1];
        m_robotCtrl.ra_v[4] = rapitch_out[1];
        m_robotCtrl.ra_v[5] = rayaw_out[1];
        ///=========================cm=========================================
        ddouble_t cmx_start[2], cmx_target[2], cmx_out[2];
        ddouble_t cmy_start[2], cmy_target[2], cmy_out[2];
        ddouble_t cmz_start[2], cmz_target[2], cmz_out[2];
        ddouble_t cmroll_start[2], cmroll_target[2], cmroll_out[2];
        ddouble_t cmpitch_start[2], cmpitch_target[2], cmpitch_out[2];
        ddouble_t cmyaw_start[2], cmyaw_target[2], cmyaw_out[2];
        /// cm[0] cm x's direction
        cmx_start[0] = m_robotCtrl.cm[0];
        cmx_start[1] = m_robotCtrl.cm_v[0]; // m_robotCtrl.ra_v[0];
        cmx_target[0] = hip_target[0];
        cmx_target[1] = 0;
        ankleCtrl.oned_analysis(cmx_out, cmx_start, cmx_target, m_robotCtrl.num_left);

        /// cm[1] cm y's direction
        cmy_start[0] = m_robotCtrl.cm[1];
        cmy_start[1] = m_robotCtrl.cm_v[1]; // m_robotCtrl.ra_v[0];
        cmy_target[0] = hip_target[1];
        cmy_target[1] = 0;
        ankleCtrl.oned_analysis(cmy_out, cmy_start, cmy_target, m_robotCtrl.num_left);

        /// cm[2] cm z's direction
        cmz_start[0] = m_robotCtrl.cm[2];
        cmz_start[1] = m_robotCtrl.cm_v[2]; // m_robotCtrl.ra_v[0];
        cmz_target[0] = hip_target[2];
        cmz_target[1] = 0;
        ankleCtrl.oned_analysis(cmz_out, cmz_start, cmz_target, m_robotCtrl.num_left);

        /// cm[3] center of mass roll
        cmroll_start[0] = m_robotCtrl.cm[3];
        cmroll_start[1] = m_robotCtrl.cm_v[3]; // m_robotCtrl.cm_v[0];
        cmroll_target[0] = hip_target[3];
        cmroll_target[1] = 0;
        rpyCtrl.oned_analysis(cmroll_out, cmroll_start, cmroll_target, m_robotCtrl.num_left);
        /// cm[4] center of mass pitch
        cmpitch_start[0] = m_robotCtrl.cm[4];
        cmpitch_start[1] = m_robotCtrl.cm_v[4];
        cmpitch_target[0] = hip_target[4];
        cmpitch_target[1] = 0;
        rpyCtrl.oned_analysis(cmpitch_out, cmpitch_start, cmpitch_target, m_robotCtrl.num_left);
        /// cm[5] center of mass yaw
        cmyaw_start[0] = m_robotCtrl.cm[5];
        cmyaw_start[1] = m_robotCtrl.cm_v[5];
        cmyaw_target[0] = hip_target[5];
        cmyaw_target[1] = 0;
        rpyCtrl.oned_analysis(cmyaw_out, cmyaw_start, cmyaw_target, m_robotCtrl.num_left);
        /// cm update
        if (m_robotCtrl.auto_cm[0]) {
            cm[0] = cmx_out[0];
            m_robotCtrl.cm_v[0] = cmx_out[1];
        } else {
            cm[0] = hip_target[0];
            m_robotCtrl.cm_v[1] = 0;
        }
        if (m_robotCtrl.auto_cm[1]) {
            cm[1] = cmy_out[0];
            m_robotCtrl.cm_v[1] = cmy_out[1];
        } else {
            cm[1] = hip_target[1];
            m_robotCtrl.cm_v[1] = 0;
        }
        if (m_robotCtrl.auto_cm[2]) {
            cm[2] = cmz_out[0];
            m_robotCtrl.cm_v[2] = cmz_out[1];
        } else {
            cm[2] = hip_target[2];
            m_robotCtrl.cm_v[2] = 0;
        }
        cm[3] = cmroll_out[0];
        cm[4] = cmpitch_out[0];
        cm[5] = cmyaw_out[0];

        /// m_robotCtrl update cm[0-5] cm_v[0-5]
        for (int i = 0; i < 6; i++) {
            m_robotCtrl.cm[i] = cm[i];
        }
        m_robotCtrl.cm_v[3] = cmroll_out[1];
        m_robotCtrl.cm_v[4] = cmpitch_out[1];
        m_robotCtrl.cm_v[5] = cmyaw_out[1];

        /// leftnum == 1,离线步态的做法，此时将坐标系转换到新的支撑腿上
        if (m_robotCtrl.num_left == 1 && m_robotCtrl.supportStatus != DOUBLE_BASED) // bug
        {
            ddouble_t c1, s1, S_ra[2], S_la[2], S_cm[2], a_tmp, S_rayaw, S_layaw;
            s1 = sin(cm[5] * M_PI / 180);
            c1 = cos(cm[5] * M_PI / 180);
            if (m_robotCtrl.supportStatus == LEFT_BASED) {
                a_tmp = c1 * ra[0] - (c1 * cm[0] + cm[1] * s1) + s1 * ra[1];
                S_ra[0] = 0;
                S_ra[1] = c1 * ra[1] - (c1 * cm[1] - cm[0] * s1) - s1 * ra[0];
                S_la[0] = (c1 * (la[0]) - (c1 * cm[0] + cm[1] * s1) + (s1 * (la[1]))) - a_tmp;
                S_la[1] = (c1 * (la[1]) - (c1 * cm[1] - cm[0] * s1) - (s1 * (la[0])));
                S_rayaw = 0;
                S_layaw = la[5] - ra[5];
            } else {
                a_tmp = c1 * la[0] - (c1 * cm[0] + cm[1] * s1) + s1 * la[1];
                S_ra[0] = (c1 * (ra[0]) - (c1 * cm[0] + cm[1] * s1) + (s1 * (ra[1]))) - a_tmp;
                S_ra[1] = (c1 * (ra[1]) - (c1 * cm[1] - cm[0] * s1) - (s1 * (ra[0])));
                S_la[0] = 0;
                S_la[1] = (c1 * (la[1]) - (c1 * cm[1] - cm[0] * s1) - (s1 * (la[0])));
                S_layaw = 0;
                S_rayaw = ra[5] - la[5];
            }

            S_cm[0] = -a_tmp;
            S_cm[1] = 0;

            for (int i = 0; i < 2; i++) {
                m_robotCtrl.la[i] = S_la[i];
                m_robotCtrl.ra[i] = S_ra[i];
                m_robotCtrl.cm[i] = S_cm[i];
            }
            m_robotCtrl.la[5] = S_layaw;
            m_robotCtrl.ra[5] = S_rayaw;
            m_robotCtrl.cm[5] = 0;
        }

        /// real cm must add cm_dxy
        ddouble_t cmdx_start[2], cmdx_target[2], cmdx_out[2];
        ddouble_t cmdy_start[2], cmdy_target[2], cmdy_out[2];
        /// cmdxy[0] cm x's direction offset
        cmdx_start[0] = m_robotCtrl.cm_dxy[0];
        cmdx_start[1] = m_robotCtrl.cm_dxy_v[0]; // m_robotCtrl.ra_v[0];
        cmdx_target[0] = targetCtrl.cm_dxy[0];
        cmdx_target[1] = 0;
        int num_left_cmdx;
        int tmp_10_cmdx;
        tmp_10_cmdx = (1 - 1) * RobotPara::stepnum; // TODO
        num_left_cmdx = m_robotCtrl.num_left - tmp_10_cmdx; // abs
        if (num_left_cmdx <= 0)
            num_left_cmdx = 0;

        ankleCtrl.oned_analysis(cmdx_out, cmdx_start, cmdx_target, num_left_cmdx);
        /// cmdxy[1] cm y's direction offset
        cmdy_start[0] = m_robotCtrl.cm_dxy[1];
        cmdy_start[1] = m_robotCtrl.cm_dxy_v[1]; // m_robotCtrl.cm_v[0];
        cmdy_target[0] = targetCtrl.cm_dxy[1];
        cmdy_target[1] = 0;
        rpyCtrl.oned_analysis(cmdy_out, cmdy_start, cmdy_target, m_robotCtrl.num_left);
        m_robotCtrl.cm_dxy[0] = cmdx_out[0];
        m_robotCtrl.cm_dxy[1] = cmdy_out[0];
        m_robotCtrl.cm_dxy_v[0] = cmdx_out[1];
        m_robotCtrl.cm_dxy_v[1] = cmdy_out[1];

        /// 将坐标系转换到胸腔坐标系上并增加质心x方向和y方向的偏移量
        ddouble_t s2, c2;
        for (int i = 0; i < 3; i++) {
            H_ra[i + 3] = ra[i + 3] * M_PI / 180;
            H_la[i + 3] = la[i + 3] * M_PI / 180;
            H_cm[i + 3] = cm[i + 3] * M_PI / 180;
        }
        s2 = sin(H_cm[5]);
        c2 = cos(H_cm[5]);

        H_ra[0] = -(c2 * cm[0] - c2 * ra[0] + cm[1] * s2 - ra[1] * s2);
        H_ra[1] = -(c2 * cm[1] - c2 * ra[1] - cm[0] * s2 + ra[0] * s2);
        H_ra[2] = ra[2] - cm[2];
        H_la[0] = -(c2 * cm[0] - c2 * la[0] + cm[1] * s2 - la[1] * s2);
        H_la[1] = -(c2 * cm[1] - c2 * la[1] - cm[0] * s2 + la[0] * s2);
        H_la[2] = la[2] - cm[2];

        H_cm[0] = 0 + cmdx_out[0];
        H_cm[1] = 0 + cmdy_out[0];
        H_cm[2] = 0;

        H_ra[5] = H_ra[5] - H_cm[5];
        H_la[5] = H_la[5] - H_cm[5];
        H_cm[5] = 0;

        for (int i = 0; i < 6; i++) {
            H_rh[i] = rh[i];
            H_lh[i] = lh[i];
        }

        // cout<<H_ra[0]<<" "<<H_ra[1]<<" "<<H_ra[2]<<endl;
    }

    // cm[0] = cm[0] + m_cm_dx;//special last deal
    // cm[1] = cm[1] + m_cm_dy;//special

    ddouble_t *rq, *lq;

    /// IK function
    rq = get_Angle(H_cm, H_ra, H_rh, 0);
    lq = get_Angle(H_cm, H_la, H_lh, 1);

    /// convert to dataArray
    for (int i = 0; i < MOTORNUM; i++)
        dataArray[i] = 0;

    q2data(dataArray, lq, rq);
    // template feature
    // dataArray[4-1] = 0;
    // dataArray[11-1] = 0;
}

/*TODO*/
/*Static motion may can be One Class to easy coding*/
//机器人的第一步，一种tricky 的实现方法，使机器人身体摆动起来后进入踏步状态。
void
HumanRobot::dofirstStep()
{
    RobotCtrl ori;
    m_robotCtrl.supportStatus = RIGHT_BASED;
    m_robotCtrl.setWalkMode();
    RobotCtrl targetCtrl(m_robotCtrl);
    // ddouble_t *l_s = new ddouble_t[firstStep_length];

    /* to do const length */
    ddouble_t ahml_ori, ahmr_ori;
    ahml_ori = RobotPara::ah_ml_zero;
    ahmr_ori = RobotPara::ah_mr_zero;

    // cout<< (*m_manager->prior_gaitState).getStateType() <<endl;
    ddouble_t ankle_distance_big = RobotPara::ankle_distance + 2 * RobotPara::yzmp + RobotPara::Ankle_dev_l - RobotPara::Ankle_dev_r; //机器人走路时候即最大情况下的两脚间距
    ddouble_t ankle_distance_small = ori.la[1] - ori.ra[1]; //机器人最开始的两脚间距，即髋关节的间距
    // cout<< "la - ra[1] " << m_robotCtrl.la[1] - m_robotCtrl.ra[1] << " " <<
    // ankle_distance_small << " " << ankle_distance_big <<endl;

    vector<ddouble_t> x_cm, y_cm;
    x_cm.push_back(ankle_distance_small);
    x_cm.push_back(ankle_distance_big);
    y_cm.push_back(RobotPara::stand2crouch2step_cm);
    y_cm.push_back(RobotPara::other2crouch2step_cm);
    //当前机器人质心的偏移量与两脚间距呈线性假设
    ddouble_t cm_excute = getPieceWise(x_cm, y_cm, m_robotCtrl.la[1] - m_robotCtrl.ra[1]);

    // m_robotCtrl.ra[1]));

    firstStep_method = RobotPara::other2crouch2step;
    // l_s = curveCreate(0, 0, RobotPara::other2crouch2step_height,
    // firstStep_length);
    ddouble_t* cm_s = curveCreate(0, 0, cm_excute, firstStep_length);
    RobotPara::ah_ml_zero = RobotPara::other2crouch2step_height;
    RobotPara::ah_mr_zero = RobotPara::other2crouch2step_height;

    targetCtrl.cm[2] = RobotPara::hipheight;

    int* dataArray = new int[MOTORNUM];
    for (int i = 0; i < firstStep_length; i++) {
        targetCtrl.la[2] = 0; // l_s[i];
        targetCtrl.cm[1] = cm_s[i]; // firstStep_data[firstStep_method][i];
        m_robotCtrl.num_left = firstStep_length - i;
        getAngle_serial(targetCtrl, dataArray, 1);
        doTxTask(dataArray);
    }
    m_robotCtrl.supportStatus = LEFT_BASED;

    for (int i = 0; i < firstStep_method; i++) // template use for firststep stepnum
        runWalk(0, 0, 0);
    // runWalk(0, 0, 0.5);
    // runWalk(0, 0, 0.5);
    RobotPara::ah_ml_zero = ahml_ori;
    RobotPara::ah_mr_zero = ahmr_ori;
}
void
HumanRobot::doCrouchFromStandMotor(const int stepnum_)
{
    // cout<<"motor"<<endl;
    vector<ddouble_t*> motors;
    // motors.push_back(new ddouble_t[stepnum_]);

    initdata_ = m_status->getMotorinit();
    for (int i = 0; i < MOTORNUM; i++) {
        ddouble_t* tmp; // = new ddouble_t[stepnum_];
        tmp = curveCreate(m_robotCtrl.dataArray[i], 0, 0, stepnum_);
        // tmp = curveCreate(m_robotCtrl.dataArray[i], 0, i,stepnum_);
        motors.push_back(tmp);
        // cout<< m_robotCtrl.dataArray[i]<<endl;
    }
    for (int i = 0; i < stepnum_; i++) {
        int* motionData = new int[MOTORNUM];
        for (int j = 0; j < MOTORNUM; j++) {
            motionData[j] = motors[j][i];
        }
        doTxTask(motionData);
    }
    RobotCtrl targetCtrl;
    m_robotCtrl = targetCtrl;
    // getchar();
    // cout<< motors[MOTORNUM-2][stepnum_-1]<<endl;
}

void
HumanRobot::doCrouchFromStand(const int stepnum_)
{ //
    RobotCtrl targetCtrl(m_robotCtrl);
    m_robotCtrl.setAutoMode();
    // ankle
    if (*m_manager->gaitState == SETUPFRONTDOWN || *m_manager->gaitState == SETUPBACKDOWN || *m_manager->gaitState == SETUPLEFTDOWN || *m_manager->gaitState == LEFTKICK ||
        *m_manager->gaitState == KICK || *m_manager->gaitState == SETUPRIGHTDOWN) {
        targetCtrl.la[0] = 0;
        targetCtrl.la[1] = RobotPara::hip_distance / 2 + RobotPara::yzmp;
        ;
        targetCtrl.ra[0] = 0;
        targetCtrl.ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp;
        ;
    }

    targetCtrl.la[2] = 0;
    targetCtrl.la[3] = RobotPara::la_r;
    targetCtrl.la[4] = RobotPara::la_p;
    targetCtrl.la[5] = 0;

    targetCtrl.ra[2] = 0;
    targetCtrl.ra[3] = RobotPara::ra_r;
    targetCtrl.ra[4] = RobotPara::ra_p;
    targetCtrl.ra[5] = 0;

    // cm
    targetCtrl.cm[0] = 0;
    targetCtrl.cm[2] = RobotPara::hipheight;
    targetCtrl.cm[3] = RobotPara::cm_r;
    targetCtrl.cm[4] = RobotPara::cm_p;
    targetCtrl.cm[5] = RobotPara::cm_y;

    targetCtrl.cm_dxy[0] = RobotPara::cm_dx;
    targetCtrl.cm_dxy[1] = RobotPara::cm_dy;

    // arm
    targetCtrl.lh[0] = RobotPara::arm_crouch_p;
    targetCtrl.rh[0] = RobotPara::arm_crouch_p;
    targetCtrl.lh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.rh[1] = RobotPara::arm_crouch_theta;
    int* dataArray = new int[MOTORNUM]; //[MOTORNUM];
    int crouch_stepnum;
    crouch_stepnum = stepnum_ * 2;

    for (int i = 0; i < crouch_stepnum; i++) {
        m_robotCtrl.num_left = crouch_stepnum - i;

        getAngle_serial(targetCtrl, dataArray, 1);
        doTxTask(dataArray);
    }
    m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void
HumanRobot::doStandFromCrouch(const int stepnum_)
{
    ROS_DEBUG("doStandFromCrouch %d", stepnum_);
    RobotCtrl targetCtrl(m_robotCtrl);
    m_robotCtrl.setAutoMode();

    // ankle
    if (*m_manager->gaitState == SETUPFRONTDOWN || *m_manager->gaitState == SETUPBACKDOWN || *m_manager->gaitState == SETUPLEFTDOWN || *m_manager->gaitState == SETUPRIGHTDOWN) {
        targetCtrl.la[0] = 0;
        targetCtrl.la[1] = 0;
        targetCtrl.ra[0] = 0;
        targetCtrl.ra[1] = 0;
    }

    targetCtrl.la[2] = 0;
    targetCtrl.la[3] = RobotPara::la_r;
    targetCtrl.la[4] = RobotPara::la_p;
    targetCtrl.la[5] = 0;

    targetCtrl.ra[2] = 0;
    targetCtrl.ra[3] = RobotPara::ra_r;
    targetCtrl.ra[4] = RobotPara::ra_p;
    targetCtrl.ra[5] = 0;

    // cm
    targetCtrl.cm[0] = 0;
    targetCtrl.cm[2] = RobotPara::upper_leg + RobotPara::lower_leg;
    targetCtrl.cm[3] = 0; // RobotPara::cm_r;
    targetCtrl.cm[4] = 0; // RobotPara::cm_p;
    targetCtrl.cm[5] = 0; // RobotPara::cm_y;

    targetCtrl.cm_dxy[0] = 0; // RobotPara::cm_dx;
    targetCtrl.cm_dxy[1] = 0; // RobotPara::cm_dy;

    // arm
    targetCtrl.lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
    targetCtrl.rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
    targetCtrl.lh[1] = 0;
    targetCtrl.rh[1] = 0;

    int* dataArray = new int[MOTORNUM]; //[MOTORNUM]; // TODO(MWX): MEMORY LEAK
    int crouch_stepnum;
    crouch_stepnum = stepnum_;
    for (int i = 0; i < crouch_stepnum; i++) {
        m_robotCtrl.num_left = crouch_stepnum - i;
        getAngle_serial(targetCtrl, dataArray, 1);
        doTxTask(dataArray);
    }
    m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void
HumanRobot::staticEntry()
{
    if (m_robotCtrl.supportStatus != DOUBLE_BASED) {
        // cout << "[Crouch] entry() do 2 steps. " << endl;
        runWalk(0, 0, 0 / 2);
        runWalk(0, 0, 0 / 2);
    } else {
        // robot->runWalk(0, 0, 0 / 2);
        // cout << "[Crouch] entry() do nothing. " << endl;
    }
    m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void
HumanRobot::staticExit()
{
    if (*m_manager->goal_gaitState == STANDUP || *m_manager->goal_gaitState == KICK || *m_manager->goal_gaitState == GOALIEMID || *m_manager->goal_gaitState == LEFTKICK ||
        *m_manager->goal_gaitState == CROUCH || *m_manager->goal_gaitState == GOALIELEFT || *m_manager->goal_gaitState == GOALIERIGHT || *m_manager->goal_gaitState == SETUPBACKDOWN ||
        *m_manager->goal_gaitState == SETUPFRONTDOWN || *m_manager->goal_gaitState == SETUPLEFTDOWN || *m_manager->goal_gaitState == SETUPRIGHTDOWN) {
        ROS_DEBUG("[Crouch] exit() do nothing.");
    } else {
        ROS_DEBUG("Do first step");
        dofirstStep();
        ROS_DEBUG("First step done");
        for (int i = 0; i < RobotPara::staticExit_num; i++) {
            runWalk(0, 0, 0);
        }

        ROS_DEBUG("Crouch exit() do  first step");
    }
}
