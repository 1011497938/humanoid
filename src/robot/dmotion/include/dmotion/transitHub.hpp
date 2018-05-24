#pragma once
#include <memory>
#include <string>
#include <ros/ros.h>

#include "MotionShareData.hpp"
#include "RobotStatus.hpp"
using std::string;
/*
** Data packets enum
** PACKET_MOTION 	 	= { 20 values, i.e. 20 motors's goal position }
** PACKET_PING 		 	= { NULL }
** PACKET_CHECK 		= {
**                      1st refers to the motor id,
**								      2nd refers to the control word
**                    }
** PACKET_SET_INIT  = {
**                      1st refers to to the motor id,
**  								    2nd refers to the goal position
**                    }
** PACKET_GYRO_PRESS= {
**                      1st refers to gyro if it's 0, to press if it's 1,
**                      2nd refers to return command if it's 1, to NOT
**								      return command if it's 0
**                    }
** PACKET_LOCK			= {
**                      1st refers to the motor id,
**								      2nd refers to lock if it's 1, to unlock if it's 0
**                    }
*/

enum packet_types
{
    PACKET_NULL, // 0
    PACKET_MOTION, // 1
    PACKET_PING, // 2
    PACKET_CHECK, // 3
    PACKET_SET_INIT, // 4
    PACKET_GYRO_PRESS, // 5
    PACKET_LOCK, // 6
    PACKET_INITIAL, // 7
};

class transitHub
{
  private:
    ros::NodeHandle* m_nh;
    RobotStatus* m_status;
    bool m_bOpen;
    int m_ifd;
    long m_gaitCycle;
    int comNumber;
    /* com name for ttyUSB0 */
    string comName;
    int baudRate;
    int m_imu_prepare_time;
    initdataDebug initdata;

    ddouble_t m_gyroscope_correct;
    string m_imu_version;
    int m_robot_version;

    /* the return value refers to the actual number of the transmitted data */
    void transmit(int* data, packet_types type = PACKET_MOTION);
    bool portOpen(); /* /dev/ttyS0 or /dev/ttyUSB0 */
    bool isOpen();
    bool portClose();

    /* the return value refers to the actual number of the reveived data */
    int receive(char* str, int length);
    /* generate the result string(str) from the input(data),
     * according to the packet type(type), return the length of the string
     */
    size_t makeStdData(char* str, int* data, packet_types type);

    /**
     * data sharing with other modules
     **/
    CompassData compass;
    GyroData gyroscope;
    ros::Time t_time;

    bool m_simulation;

    // to do: private functions
  public:
    transitHub(ros::NodeHandle* nh, RobotStatus* rs);
    ~transitHub();
    void readOptions();
    void update_initdata();

    void gypdatatransform(char* temp);
    void feetdatatransform(char* temp);
    void updateInitData();

    /* basic task of receiving process */
    void doRx(char* recv = NULL, int failureCount = 0);

    /* basic task of transmitting process */
    void doLoopTx(int* body = NULL, int* camera = NULL);

    void doCheckTx(int id = 0, bool pos = false, bool temp = false, bool vol = false, bool load = false);

    // long getGaitTime();
    // void clearGaitTime();
    // void setRobotStatus(robotStatus *status);
};
