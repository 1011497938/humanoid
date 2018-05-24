#include "dmotion/GaitStateLib/GaitStateGoalie.hpp"

GaitStateGoalie::GaitStateGoalie(I_HumanRobot* robot)
  : GaitStateBase(GOALIEMID, robot), m_goalie_left(true)
{
    loadGaitFile();
}

GaitStateGoalie::~GaitStateGoalie()
{
}

void
GaitStateGoalie::loadGaitFile()
{
    ROS_DEBUG("Golie reload gait file");
    length = robot->loadGaitFile("goalie", data);
    // lengthL_ = robot->loadGaitFile("leftKick", dataL_);
}

// void GaitStateGoalie::readOptions(
//     const boost::program_options::variables_map& config) {
//   m_stepnum = config["robot.goalie_stepnum"].as<int>();
//   m_goalie_bool = config["robot.goalie_bool"].as<bool>();
//   m_sleeptime = config["robot.goalie_time"].as<int>();
//   m_zf_15 = config["motor.15.zf"].as<int>();
//   m_zf_17 = config["motor.17.zf"].as<int>();

// }

void
GaitStateGoalie::entry()
{
    std::cout << "start" << std::endl;
    robot->m_robotCtrl.supportStatus = DOUBLE_BASED;

    int step = 150;
    int* dataArray = new int[MOTORNUM];

    //robot->doStandFromCrouch(10);

    RobotCtrl target_robotCtrl = robot->m_robotCtrl;

    for (int i = 0; i < length; i++) {
        step = data[0][i];
        if (step > 1000)
            break;

        //if (i == 2) usleep(2000000);



        target_robotCtrl.cm[0] = data[1][i];
        target_robotCtrl.cm[2] = data[2][i];
        target_robotCtrl.cm[4] = data[3][i];

        target_robotCtrl.lh[0] = data[4][i];
        target_robotCtrl.rh[0] = data[4][i];

        if (m_goalie_left)
        {
            /*target_robotCtrl.la[1] = data[4][i];
            target_robotCtrl.ra[1] = data[5][i];
            target_robotCtrl.la[2] = data[6][i];
            target_robotCtrl.ra[2] = data[7][i];*/
            target_robotCtrl.lh[1] = data[5][i];
            target_robotCtrl.rh[1] = data[5][i];

            target_robotCtrl.ra[4] = data[6][i];
            target_robotCtrl.la[4] = data[6][i];
        }
        else
        {
            /*target_robotCtrl.la[1] = -data[5][i];
            target_robotCtrl.ra[1] = data[4][i];
            target_robotCtrl.la[2] = data[7][i];
            target_robotCtrl.ra[2] = data[6][i];*/
            target_robotCtrl.lh[1] = -data[5][i];
            target_robotCtrl.rh[1] = -data[5][i];

            target_robotCtrl.ra[4] = -data[6][i];
            target_robotCtrl.la[4] = -data[6][i];
        }

        //target_robotCtrl.cm[0] = data[8][i];
        //target_robotCtrl.cm[2] = data[9][i];
        //target_robotCtrl.cm[4] = data[10][i];

        for (int j = 0; j < step; j++) {
            robot->m_robotCtrl.num_left = step - j;
            robot->getAngle_serial(target_robotCtrl, dataArray, 1);
            robot->doTxTask(dataArray);
        }

//        if(i == 0 ) usleep(3000000);
        if (i == 0) {
            auto start = ros::Time::now();
            while(true) {
                auto elapsed = (ros::Time::now() - start).toSec();
                if(elapsed > data[7][0])
                    break;
                robot->doTxTask(dataArray);
            }
        } else {
            robot->doTxTask(dataArray);
        }

    }
    //robot->doCrouchFromStand(100);
}

void
GaitStateGoalie::exit()
{
    // robot->doCrouchFromStand(RobotPara::stepnum * 2); // crouch first
    robot->staticExit();
    // std::cout <<"stand up exit" <<std::endl;
}

void
GaitStateGoalie::execute()
{
    // robot->doCrouchFromStand(m_stepnum);
    // doRecover();
}

void
GaitStateGoalie::doLiftbothHand()
{
}

void
GaitStateGoalie::doGoalieMid()
{
}

void
GaitStateGoalie::doRecover()
{
}

void
GaitStateGoalie::setLeftGoalie()
{
    m_goalie_left = true;
}

void
GaitStateGoalie::setRightGoalie()
{
    m_goalie_left = false;
}
