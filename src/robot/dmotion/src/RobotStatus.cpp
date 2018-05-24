#include "dmotion/RobotStatus.hpp"
#include <fstream>
#include <ros/ros.h>

using namespace std;
RobotStatus::RobotStatus(ros::NodeHandle* nh)
  : m_nh(nh)
{
    RobotPara::update(m_nh); // FUCK(MWX): MUST call before everyone
    readOptions();
    initMotor();
    m_last_angle_z = 0;
    m_isRun = false;
    m_comInfo = new comEstimator();
    compass_filter = One_D_Filter(1000);
    //----add by yyj----
    m_imu_filter.setWorldFrame(WorldFrame::NED);
    m_imu_filter.setDriftBiasGain(0.0);
    m_imu_filter.setAlgorithmGain(0.1);
    gyro_x_avg = 0;
    gyro_y_avg = 0;
    gyro_z_avg = 0;
    imu_roll_bias = 0;
    imu_pitch_bias = 0;
    imu_yaw_bias = 0;
    imu_init_cnt = 0;
    //------------------
}

RobotStatus::~RobotStatus()
{
}

angledataDebug
RobotStatus::getAngledata()
{
    angledataDebug temp = m_angledata;
    return temp;
}

GyroData
RobotStatus::getGdata()
{
    return m_gypdata;
}

void
RobotStatus::resetGyro(){
    imu_initialized = false;
}

void
RobotStatus::setGdata(GyroData gdata)
{
    m_gypdata = gdata;
    // update body angle
    m_angledata.angleX += gdata.GYPO[0] * MOTION::sampletime;
    m_angledata.angleY += gdata.GYPO[1] * MOTION::sampletime;
    m_angledata.angleZ += gdata.GYPO[2] * MOTION::sampletime;
    m_angledata.corresCycle = gdata.corresCycle;
    // update offset
    m_offset.offsetX += gdata.ACCL[0] * MOTION::sampletime * MOTION::sampletime * 100;
    m_offset.offsetY += gdata.ACCL[1] * MOTION::sampletime * MOTION::sampletime * 100;
    m_offset.corresCycle = gdata.corresCycle;


    //----add by yyj----
    geometry_msgs::Vector3 ang_vel;
    ang_vel.x = static_cast<float>(gdata.GYPO[0]) / 180 * M_PI;
    ang_vel.y = static_cast<float>(gdata.GYPO[1]) / 180 * M_PI;
    ang_vel.z = static_cast<float>(gdata.GYPO[2]) / 180 * M_PI;
    geometry_msgs::Vector3 lin_acc;
    lin_acc.x = static_cast<float>(gdata.ACCL[0]);
    lin_acc.y = static_cast<float>(gdata.ACCL[1]);
    lin_acc.z = static_cast<float>(gdata.ACCL[2]);


    if (!imu_initialized)
    {
        geometry_msgs::Quaternion init_q;
        StatelessOrientation::computeOrientation(WorldFrame::NED, lin_acc, init_q);
        m_imu_filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
        imu_initialized = true;
        imu_init_cnt = 0;
    }
    m_imu_filter.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                       lin_acc.x, lin_acc.y, lin_acc.z,
                                       static_cast<float>(MOTION::sampletime));
    float tmp_q0;
    float tmp_q1;
    float tmp_q2;
    float tmp_q3;
    m_imu_filter.getOrientation(tmp_q0, tmp_q1, tmp_q2, tmp_q3);
    angledataDebug tmp_rpy;
    tf2::Matrix3x3(tf2::Quaternion(tmp_q1, tmp_q2, tmp_q3, tmp_q0)).getRPY(tmp_rpy.angleX, tmp_rpy.angleY, tmp_rpy.angleZ, 0);

    // std::cout << "angledata.angleX:" << m_angledata.angleX << std::endl;
    // std::cout << "angledata.angleY:" << m_angledata.angleY << std::endl;
    // std::cout << "angledata.angleZ:" << m_angledata.angleZ << std::endl;


    imu_init_cnt++;
//    if(imu_init_cnt % 10 == 0){
//        std::cout << "imu_init_cnt    :" << imu_init_cnt << std::endl;
//        std::cout << "gdata.GYPO[0] rX:" << gdata.GYPO[0] << std::endl;
//        std::cout << "gdata.GYPO[1] pY:" << gdata.GYPO[1] << std::endl;
//        std::cout << "gdata.GYPO[2] yZ:" << gdata.GYPO[2] << std::endl;
//        std::cout << "gdata.ACCL[0] aX:" << gdata.ACCL[0] << std::endl;
//        std::cout << "gdata.ACCL[1] aY:" << gdata.ACCL[1] << std::endl;
//        std::cout << "gdata.ACCL[2] aZ:" << gdata.ACCL[2] << std::endl;
//        std::cout << "angle          X:" << m_angle_rpy.angleX / M_PI * 180 << std::endl;
//        std::cout << "angle          Y:" << m_angle_rpy.angleY / M_PI * 180 << std::endl;
//        std::cout << "angle          Z:" << m_angle_rpy.angleZ / M_PI * 180 << std::endl;
//    }

    if(imu_init_cnt < RobotPara::stepnum * 3 * imu_prepare_time){
        m_angle_rpy.angleX = 0;
        m_angle_rpy.angleY = 0;
        m_angle_rpy.angleZ = M_PI / 2;
        // imu_init_cnt++;
    } else if(imu_init_cnt == RobotPara::stepnum * 3 * imu_prepare_time){
        m_angle_rpy.angleX = 0;
        m_angle_rpy.angleY = 0;
        m_angle_rpy.angleZ = M_PI / 2;
        imu_roll_bias = tmp_rpy.angleX;
        imu_pitch_bias = tmp_rpy.angleY;
        imu_yaw_bias = tmp_rpy.angleZ;
    } else {
        m_angle_rpy.angleX = tmp_rpy.angleX - imu_roll_bias;
        m_angle_rpy.angleY = tmp_rpy.angleY - imu_pitch_bias;
        m_angle_rpy.angleZ = tmp_rpy.angleZ - imu_yaw_bias + M_PI / 2;
    }


    /*--------------------------------------
     DON‘T DELETE using for calculate bias
     ------------------------------------*/
    if(gyro_x_sum.size() <= 6000){
        gyro_x_sum.push_back(gdata.GYPO[0]);
    }else{
        gyro_x_sum.pop_front();
        gyro_x_sum.push_back(gdata.GYPO[0]);
    }
    if(gyro_y_sum.size() <= 6000){
      gyro_y_sum.push_back(gdata.GYPO[1]);
    }else{
        gyro_y_sum.pop_front();
        gyro_y_sum.push_back(gdata.GYPO[1]);
    }
    if(gyro_z_sum.size() <= 6000){
        gyro_z_sum.push_back(gdata.GYPO[2]);
    }else{
        gyro_z_sum.pop_front();
        gyro_z_sum.push_back(gdata.GYPO[2]);
    }

    float tmp_x_sum = 0;
    for (size_t i = 0; i < gyro_x_sum.size(); i++) {
        tmp_x_sum += gyro_x_sum[i];
    }
    gyro_x_avg = tmp_x_sum / gyro_x_sum.size();

    float tmp_y_sum = 0;
    for (size_t i = 0; i < gyro_y_sum.size(); i++) {
        tmp_y_sum += gyro_y_sum[i];
    }
    gyro_y_avg = tmp_y_sum / gyro_y_sum.size();

    float tmp_z_sum = 0;
    for (size_t i = 0; i < gyro_z_sum.size(); i++) {
        tmp_z_sum += gyro_z_sum[i];
    }
    gyro_z_avg = tmp_z_sum / gyro_z_sum.size();

//    if(imu_init_cnt % 10 == 0){
//        std::cout << "gyro_x_avg      :" << gyro_x_avg << std::endl;
//        std::cout << "gyro_y_avg      :" << gyro_y_avg << std::endl;
//        std::cout << "gyro_z_avg      :" << gyro_z_avg << std::endl;
//    }

    // ----------------end of add------------
}

void
RobotStatus::updateEularAngle()
{
    VecPos eular;
    GyroData cur_gyro_data = m_gypdata;
    m_comInfo->m_filter->estimate(cur_gyro_data.GYPO[0], cur_gyro_data.GYPO[1], cur_gyro_data.GYPO[2], cur_gyro_data.ACCL[0], cur_gyro_data.ACCL[1], cur_gyro_data.ACCL[2], MOTION::sampletime);
    eular.m_x = m_comInfo->m_filter->eular[0];
    eular.m_y = m_comInfo->m_filter->eular[1];
    m_eular_deque.push_front(eular);
    if (m_eular_deque.size() > 50) // 1s
    {
        m_eular_deque.pop_back();
    }
    return;
}

stabilityStatus
RobotStatus::checkStableState()
{
    /// get eular angle
    VecPos eular = m_eular_deque.front();
    float temp;

    temp = eular.m_x;
    eular.m_x = eular.m_y;
    eular.m_y = temp * -1;

    // if( c_gyro_type =="MPU6000")
    // {
    //     if(c_robot_version == "2012")
    //     {
    //         temp = eular.m_x;
    //         eular.m_x = eular.m_y;
    //         eular.m_y = temp*-1;
    //     }
    //     else if(c_robot_version == "2012.5")
    //     {
    //         eular.m_x*=-1;
    //         eular.m_y*=-1;
    //     }
    //     else// "2013"
    //     {
    //         eular.m_x*=-1;
    //         eular.m_y*=-1;
    //     }
    // }

    eular.m_x *= -1;
    eular.m_y *= -1;

    eular.m_x *= (180 / M_PI);
    eular.m_y *= (180 / M_PI);
    static int frontcount, backcount, leftcount, rightcount, stablecount;


    if (eular.m_x < -5 || eular.m_x > 25 || abs(eular.m_y) > 28) {

        stablecount = 0;
        // m_desPlat.m_x = 0;
        // m_desPlat.m_y = 0;
        // m_PlatRotateStep = 12;

        if (eular.m_x > 60) {
            frontcount++;
            m_bStable = unstable;
        } else {
            frontcount = 0;
        }

        if (eular.m_x < -60) {
            backcount++;
            m_bStable = unstable;
        } else {
            backcount = 0;
        }

        if (eular.m_y > 60) {
            rightcount++;
            m_bStable = unstable;
        } else {
            rightcount = 0;
        }

        if (eular.m_y < -60) {
            leftcount++;
            m_bStable = unstable;
        } else {
            leftcount = 0;
        }
    } else {
        stablecount++;
        frontcount = backcount = leftcount = rightcount = 0;
    }

    if (frontcount > 10) {
        m_bStable = frontdown;
    } else if (backcount > 10) {
        m_bStable = backdown;
    } else if (leftcount > 10) {
        m_bStable = leftdown;
    } else if (rightcount > 10) {
        m_bStable = rightdown;
    } else if (stablecount > 10) {
        m_bStable = stable;
    }
    return m_bStable;
}

void
RobotStatus::updateDeltaDist(float dx, float dy, float dt)
{
//    int k = -1;
//    deltaBodyAngle = k * deltaBodyAngle;
//    float rx = D.m_x / (deltaBodyAngle / 180 * M_PI);
//    float ry = D.m_y / (deltaBodyAngle / 180 * M_PI);
//    if (deltaBodyAngle == 0) {
//        m_deltaDist.m_x += D.m_x;
//        m_deltaDist.m_y += D.m_y;
//    } else if (m_deltaDist.m_angle == 0 && m_deltaDist.m_x == 0 && m_deltaDist.m_y == 0) {
//        m_deltaDist.m_x = rx * sin(deltaBodyAngle / 180 * M_PI) + ry * (1 - cos(deltaBodyAngle / 180 * M_PI));
//        m_deltaDist.m_y = rx * (1 - cos(deltaBodyAngle / 180 * M_PI)) + ry * sin(deltaBodyAngle / 180 * M_PI);
//        m_deltaDist.m_angle = deltaBodyAngle;
//    } else {
//        VecPos addValue =
//          VecPos(rx * sin(deltaBodyAngle / 180 * M_PI) + ry * (1 - cos(deltaBodyAngle / 180 * M_PI)), rx * (1 - cos(deltaBodyAngle / 180 * M_PI)) + ry * sin(deltaBodyAngle / 180 * M_PI));
//        addValue.rotate(m_deltaDist.m_angle);
//        m_deltaDist.m_x += addValue.m_x;
//        m_deltaDist.m_y += addValue.m_y;
//        m_deltaDist.m_angle += deltaBodyAngle;
//        auto t = m_deltaDist.m_angle;
    auto t = m_deltaDist.m_angle;
    auto sin_ = sin(t / 180.0 * M_PI);
    auto cos_ = cos(t / 180.0 * M_PI);

    auto xp = dx * cos_ - dy * sin_;
    auto yp = dx * sin_ + dy * cos_;

    m_deltaDist.m_x += xp;
    m_deltaDist.m_y += yp;
    m_deltaDist.m_angle = AngleNormalization(m_deltaDist.m_angle + dt);
}

initdataDebug
RobotStatus::getMotorinit()
{
    return m_motorini;
}

initdataDebug
RobotStatus::getRawMotorInit()
{
    return raw_motorini;
}

void
RobotStatus::readOptions()
{
    if (!m_nh->getParam("dmotion/motor/k", k_tmp)) {
        ROS_FATAL("Get motor k error");
    }
    for (auto& k : k_tmp) {
        if (k != 4096 && k != 1024) {
            ROS_ERROR("mx? rx? or other fucking dynamixel motor? ");
        }
        k /= 360.0;
        ROS_DEBUG("motor k %lf", k);
    }
}

void
RobotStatus::initMotor()
{
    ROS_INFO("Init motor");
    vector<float> initial_tmp;
    if (!m_nh->getParam("dmotion/motor/init", initial_tmp)) {
        ROS_FATAL("Can't get initialData");
        std::terminate();
    }
    assert(initial_tmp.size() == MOTORNUM);

    for (int i = 0; i < MOTORNUM; i++) {
        raw_motorini.initial[i] = initial_tmp[i];
        m_motorini.initial[i] = (int)(k_tmp[i] * raw_motorini.initial[i]);
        ROS_INFO("init[%d]: %lf", i, initial_tmp[i]);
        if (m_motorini.initial[i] < 0 || m_motorini.initial[i] > k_tmp[i] * 360 - 1) {
            ROS_FATAL("initial data out of bound %lf", m_motorini.initial[i]);
        }
    }
}

VecPos
RobotStatus::getEularAngle()
{
    VecPos retv;
    if (m_eular_deque.size() > 0) {
        retv = m_eular_deque.front();
    } else {
        retv = VecPos(0, 0);
    }

    retv.m_x *= -1;
    retv.m_y *= -1;

    return retv;
}

void
RobotStatus::setCompassData(CompassData temp)
{
    m_compassdata = temp;
}

// clean delta data & get last delta data
deltadataDebug
RobotStatus::checkDeltaDist()
{
    deltadataDebug deltaDist = m_deltaDist;

    m_deltaDist.m_angle = 0;
    m_deltaDist.m_x = 0;
    m_deltaDist.m_y = 0;

    return deltaDist;
}

deltadataDebug
RobotStatus::getDelta() {
    return m_deltaDist;
}
