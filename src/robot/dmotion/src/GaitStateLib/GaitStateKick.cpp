#include "dmotion/GaitStateLib/GaitStateKick.hpp"
#include <ros/ros.h>

GaitStateKick::GaitStateKick(I_HumanRobot* robot, GaitStateManager* manager)
  : GaitStateBase(KICK, robot)
  , manager(manager), crouchBool_(true)
{
    loadGaitFile();
}

void
GaitStateKick::loadGaitFile()
{
    ROS_DEBUG("Kick reload gait file");
    lengthR_ = robot->loadGaitFile("rightKick", dataR_);
    lengthL_ = robot->loadGaitFile("leftKick", dataL_);
}

GaitStateKick::~GaitStateKick() = default;

void
GaitStateKick::setLeftKick() {
    m_leftKick = true;
}

void
GaitStateKick::setRightKick() {
    m_leftKick = false;
}

void
GaitStateKick::entry()
{
    robot->staticEntry();
    robot->doCrouchFromStand(30);
    robot->m_robotCtrl.setAutoMode();
    // must get the information from the gyroscope
    int step;
    // step = 150;
    // step_last = 50;
    int* dataArray = new int[MOTORNUM];
    RobotCtrl target_robotCtrl;
    target_robotCtrl = robot->m_robotCtrl;

    ddouble_t cm_advance;
    if (!m_leftKick) {
        length_ = lengthR_;
        RobotCtrl ori_robotCtrl;
        cm_advance = robot->m_robotCtrl.la[1] - ori_robotCtrl.la[1];
    } else {
        length_ = lengthL_;
        RobotCtrl ori_robotCtrl;
        cm_advance = robot->m_robotCtrl.ra[1] - ori_robotCtrl.ra[1];
    }
    // ddouble_t angle9 = 30;
    for (int i = 0; i < length_; i++) {

        // std::cout <<  i <<std::endl;
//        if (i == 3) {
//            std::cout << "sleep start" << std::endl;
//            usleep(1000000);
//            std::cout << "sleep end" << std::endl;
//        }


        if (!m_leftKick) {
            step = dataR_[0][i];
            target_robotCtrl.cm[0] = dataR_[1][i]; // 17;
            target_robotCtrl.cm[2] = dataR_[2][i]; // 6;
            target_robotCtrl.cm[3] = dataR_[3][i]; // 4;
            target_robotCtrl.cm[4] = dataR_[4][i]; // 100;
            target_robotCtrl.cm_dxy[1] = dataR_[5][i] + cm_advance;
            target_robotCtrl.ra[0] = dataR_[6][i];
            target_robotCtrl.ra[1] = dataR_[7][i];
            target_robotCtrl.ra[2] = dataR_[8][i];
            target_robotCtrl.ra[3] = dataR_[9][i];
            target_robotCtrl.ra[4] = dataR_[10][i]; // 30;
            target_robotCtrl.la[4] = dataR_[11][i]; // 30;
            // target_robotCtrl.lh[0] = dataR_[12][i];//6;
            // target_robotCtrl.lh[1] = dataR_[13][i];//-4;
            // target_robotCtrl.rh[0] = dataR_[14][i];//6;
            // target_robotCtrl.rh[1] = dataR_[15][i];//-4;
        } else {
            step = dataL_[0][i];                                    // 1
            target_robotCtrl.cm[0] = dataL_[1][i];                  // 2;
            target_robotCtrl.cm[2] = dataL_[2][i];                  // 3;
            target_robotCtrl.cm[3] = dataL_[3][i];                  // 4;
            target_robotCtrl.cm[4] = dataL_[4][i];                  // 5;
            target_robotCtrl.cm_dxy[1] = dataL_[5][i] + cm_advance; // 6
            target_robotCtrl.la[0] = dataL_[6][i];                  // 7
            target_robotCtrl.la[1] = dataL_[7][i] - 0;
            target_robotCtrl.la[2] = dataL_[8][i];
            target_robotCtrl.la[3] = dataL_[9][i];
            target_robotCtrl.la[4] = dataL_[10][i]; // 30;
            target_robotCtrl.ra[4] = dataL_[11][i]; // 30;
            // target_robotCtrl.lh[0] = dataL_[12][i];//6;
            // target_robotCtrl.lh[1] = dataL_[13][i];//-4;
            // target_robotCtrl.rh[0] = dataL_[14][i];//6;
            // target_robotCtrl.rh[1] = dataL_[15][i];//-4;
        }

        ddouble_t* dataBig;    // = new ddouble_t[step];
        ddouble_t* dataSmall;  //= new ddouble_t[step / 2];
        ddouble_t* dataBigb;   //= new ddouble_t[step];
        ddouble_t* dataSmallb; // = new ddouble_t[step / 2];

//        if (step >= 999) {
//            crouchBool_ = false;
//            break;
//        }
        for (int j = 0; j < step; j++) {
            robot->m_robotCtrl.num_left = step - j;
            robot->getAngle_serial(target_robotCtrl, dataArray, 1);
#if 1 // start kick in stage 3
            if (i == 2 && j == 0) {
                if (!m_leftKick){
                    dataBig = robot->curveCreate(dataArray[2 - 1], 0, dataArray[2 - 1] + (dataR_[15][i] / 360 * 4096), step);
                    dataSmall = robot->curveCreate(dataArray[5 - 1], 0, dataArray[5 - 1] + (dataR_[14][i] / 360 * 4096), step / 2);
                } else {
                    dataBig = robot->curveCreate(dataArray[9 - 1], 0, dataArray[9 - 1] + (dataL_[15][i] / 360 * 4096), step);
                    dataSmall = robot->curveCreate(dataArray[12 - 1], 0, dataArray[12 - 1] + (dataL_[14][i] / 360 * 4096), step / 2);
                }
            }
            if (i == 3 && j == 0) {
                if (!m_leftKick) {
                    dataBigb = robot->curveCreate(dataR_[15][i] / 360 * 4096, 0, 0, step);
                    dataSmallb = robot->curveCreate(dataR_[14][i] / 360 * 4096, 0, 0, step / 2);
                } else {
                    dataBigb = robot->curveCreate(dataL_[15][i] / 360 * 4096, 0, 0, step);
                    dataSmallb = robot->curveCreate(dataL_[14][i] / 360 * 4096, 0, 0, step / 2);
                }
            }
            if (i == 2) {
                if (!m_leftKick){
                    dataArray[2 - 1] = dataBig[j]; // dataArray[9-1] +
                    // int(dataL_[15][i]*j/step/360*4096);
                    dataArray[6 - 1] = dataArray[6 - 1] + int(dataR_[15][i] * j / step / 360 * 4096);
                } else {
                    dataArray[9 - 1] = dataBig[j]; // dataArray[9-1] +
                    // int(dataL_[15][i]*j/step/360*4096);
                    dataArray[13 - 1] = dataArray[13 - 1] + int(dataL_[15][i] * j / step / 360 * 4096);
                }
            }
            if (i == 2 && j > step / 2) {
                if (!m_leftKick)
                    dataArray[5 - 1] = dataSmall[j - step / 2 - 1]; // dataArray[12-1] +
                // int(dataL_[14][i]*j/step/360*4096);
                else
                    dataArray[12 - 1] = dataSmall[j - step / 2 - 1]; // dataArray[12-1] +
                // int(dataL_[14][i]*j/step/360*4096);

                // std::cout<< data12[j-step/2-1]<<std::endl;
            }
            if (i == 3 && j < step / 2) {
                if (!m_leftKick)
                    dataArray[5 - 1] = dataArray[5 - 1] + dataSmallb[j]; // dataArray[12-1] +
                // int(dataL_[14][i]*j/step/360*4096);
                else
                    dataArray[12 - 1] = dataArray[12 - 1] + dataSmallb[j]; // dataArray[12-1] +
                // int(dataL_[14][i]*j/step/360*4096);
            }
            if (i == 3) {
                if (!m_leftKick){
                    dataArray[2 - 1] = dataArray[2 - 1] + dataBigb[j];
                    dataArray[6 - 1] = dataArray[6 - 1] + int(dataR_[15][i] / 360 * 4096) - int(dataR_[15][i] * j / step / 360 * 4096);
                } else {
                    dataArray[9 - 1] = dataArray[9 - 1] + dataBigb[j];
                    dataArray[13 - 1] = dataArray[13 - 1] + int(dataL_[15][i] / 360 * 4096) - int(dataL_[15][i] * j / step / 360 * 4096);
                }
                // dataArray[13-1] = abs(dataArray[9-1]) - abs(dataArray[12-1]) -
                // (RobotPara::cm_p - RobotPara::la_p)*4096.0/360; dataArray[13-1] =
                // dataArray[13-1]; RobotPara::la_p);
            }


            if (i == 2 && j == step - 1) {
                auto start = ros::Time::now();
                while(true) {
                    auto elapsed = (ros::Time::now() - start).toSec();
                    if(elapsed > 1.0)
                        break;
                    robot->doTxTask(dataArray);
                }
            } else {
                robot->doTxTask(dataArray);
            }
#endif
        } // for step
        // FIXME(mwx) FUCK delete
        // if (dataBig) {
        //     delete[] dataBig;
        // }
        // if (dataBigb) {
        //     delete[] dataBigb;
        // }
        // if (dataSmallb) {
        //     delete[] dataSmallb;
        // }
        // if (dataSmall) {
        //     delete[] dataSmall;
        // }
    }
    // if (dataArray) {
    //     delete[] dataArray;
    // }
} // entry()

void
GaitStateKick::execute()
{
     robot->doCrouchFromStand(30);
}

void
GaitStateKick::exit()
{
//    if (crouchBool_)
//        robot->doCrouchFromStand(50);

    robot->staticExit();
}
