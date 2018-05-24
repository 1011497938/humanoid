#include <fcntl.h>
#include <iomanip>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>

#include "dmotion/transitHub.hpp"

// TODO(MWX): real time IO

using namespace std;

namespace {
// const int FAILURE = 0;
// const int SUCCESS = 1;
// const int NONE = 2;
// const int REPLY   = 3;
static double __attribute__((unused)) gyro_timeval[2] = { 0, 0 };
} // namespace

bool
transitHub::isOpen()
{
    return m_bOpen;
}

bool
transitHub::portClose()
{
    if (isOpen()) {
        close(m_ifd);
    }
    m_bOpen = false;
    return true;
}

void
transitHub::updateInitData()
{
    initdata = m_status->getMotorinit();
}

// todo, every 5s test if /dev/ttyUSB0 exist, if not reopen port
bool
transitHub::portOpen()
{
    portClose();

    struct termios newtio;

    /* decide if it's a file or a device */
    struct stat buf;

    if (-1 == stat((comName + std::to_string(comNumber)).c_str(), &buf)) {
        while (true) {
            // try to open video0 to video 50
            int returncode = 0;
            for (int i = 0; i < 50; i++) {
                string tmp = comName + to_string(i);
                returncode = stat(tmp.c_str(), &buf);

                if (returncode == -1) {
                    usleep(10000); // wait for 0.01s
                    continue;
                } else {
                    ROS_INFO("Found an accessible serial port dev: %s", tmp.c_str());
                    comNumber = i;
                    break;
                }
            }

            if (returncode != -1) {
                break;
            } else {
                ROS_FATAL("No serial dev accessible");
                usleep(10000);
            }
        }
    }
    auto name = comName + std::to_string(comNumber);
    ROS_INFO("Opened port: %s", name.c_str());

    m_ifd = open(name.c_str(), O_RDWR | O_NOCTTY);

    if (m_ifd < 0) {
        m_bOpen = false;
        ROS_FATAL("transitHub::portOpen failed: %s", name.c_str());
        return false;
    } else {
        ROS_INFO("Serial port open succeed: %s", name.c_str());
        m_bOpen = true;
    }

    bzero(&newtio, sizeof(newtio));
    if (57600 == baudRate) {
        newtio.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    } else if (115200 == baudRate) {
        newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    } else {
        newtio.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    }
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(m_ifd, TCIOFLUSH);
    tcsetattr(m_ifd, TCSANOW, &newtio);
    return true;
}

void
transitHub::transmit(int* data, packet_types type)
{
    char str[200];
    size_t length;
    /* if cycle < 5, send init*/
    //  if(m_gaitCycle < 20){
    //    length = makeStdData(str, data, PACKET_SET_INIT);
    //  } else {
    length = makeStdData(str, data, type);
    //  }

    int leftBytes;
    if (ioctl(m_ifd, FIONREAD, &leftBytes) < 0) {
        ROS_FATAL("ioctl FIONREAD ERROR");
    }

    /* what's done by this ? */
    if (leftBytes > 0) {
        char* tmp = new char[leftBytes + 1];
        receive(tmp, leftBytes);

        delete[] tmp;
        //    ROS_WARN("%ld receive buffer left, %d more chars!", m_gaitCycle, leftBytes);
    }
    /* with out this the time will be up to 45ms */
    if (write(m_ifd, str, length) == -1) {
        ROS_FATAL("%ld Write SerialPort Error!", m_gaitCycle);
        portClose();
        portOpen();
    }
}

size_t
transitHub::makeStdData(char* str, int* data, packet_types type)
{
    size_t len = 0;
    /* start with "0xff 0xff" */
    str[len++] = (char)(0xff);
    str[len++] = (char)(0xff);
    /* the type, i.e. 0x01,0x02,0x03... see more in pc_mega_com_protocol */
    str[len++] = type;
    if (type == PACKET_MOTION) {
        int absData[MOTORNUM];
        ROS_DEBUG("abs motion data");
        for (int i = 0; i < MOTORNUM; i++) {
            absData[i] = initdata.initial[i] + data[i];
            ROS_DEBUG("%d %d", absData[i], data[i]);
        }

        char sum = 0;
        str[len++] = 2;
        sum += str[len - 1];

        //    cout << "motion data" << endl;
        for (int i = 0; i < MOTORNUM; i++) {
            //      cout << "abs: " << absData[i] << endl;
            str[len++] = (absData[i] >> 8) & 0x00ff;
            //      printf("%02x ", ( (absData[i] >> 8) & 0x00ff ) & 0xff);
            str[len++] = absData[i] & 0x00ff;
            //      printf("%02x ", str[len] & 0xff);
            //      cout << endl;
            sum += str[len - 1] + str[len - 2];
        }

        str[len++] = (~sum & 0x00ff);
        str[len] = '\0';

        return len;
    } else if (type == PACKET_INITIAL) {
        char sum = 0;
        str[len++] = (data[0] >> 8) & 0x00ff;
        str[len++] = data[0] & 0x00ff;
        sum += str[len - 1] + str[len - 2];
        str[len++] = (~sum & 0x00ff);
        str[len] = '\0';
        return len;
    } else if (type == PACKET_PING) // 2.PACKET_PING
    {
        str[len] = '\0';
        return len;
    } else if (type == PACKET_CHECK) // 3.PACKET_CHECK
    {
        str[len++] = data[0] & 0x00ff;
        str[len++] = data[1] & 0x00ff;
        str[len] = '\0';
        return len;
    } else if (type == PACKET_SET_INIT) // 4.PACKET_SET_INIT
    {
        int len_ = 45 - 3;
        char msg[42] = "motor init";
        for (int i = 0; i < len_; i++) {
            str[len++] = msg[i];
        }
        str[len] = '\0';
        return len;
        //    char sum = 0;
        //    str[len++] = data[0] & 0x00ff;
        //    str[len++] = (data[1] >> 8) & 0x00ff;
        //    str[len++] = data[1] & 0x00ff;
        //    sum += str[len - 1] + str[len - 2] + str[len - 3];
        //    str[len++] = (~sum & 0x00ff);
        //    str[len] = '\0';
        //    return len;
    } else if (type == PACKET_GYRO_PRESS) // 5.PACKET_GYRO_PRESS
    {
        str[len++] = data[0] & 0x00ff;
        str[len++] = data[1] & 0x00ff;
        str[len] = '\0';
        return len;
    } else if (type == PACKET_LOCK) // 6.PACKET_LOCK
    {
        str[len++] = data[0] & 0x00ff;
        str[len++] = data[1] & 0x00ff;
        str[len] = '\0';
        return len;
    }
    return 0;
}

int
transitHub::receive(char* str, int length)
{

    int actualLength;
    struct termios options;
    tcgetattr(m_ifd, &options);
    options.c_cc[VMIN] = length;
    options.c_cc[VTIME] = 1;
    tcsetattr(m_ifd, TCSANOW, &options);
    actualLength = read(m_ifd, str, length);
    str[actualLength] = '\0';

    //  printf("%02x ", str[0] & 0xff);
    static long currentTime = -1;
    if (m_gaitCycle != currentTime) {
        currentTime = m_gaitCycle;
    }
    //  cout << __func__ << " " << currentTime << " actual recv length:  "
    //  << actualLength << endl;
    //  for (int i = 0; i < actualLength; i++) {
    //    if ((str[i] & 0xf0) == 0) {
    //      cout << static_cast<unsigned int>(static_cast<unsigned char>(str[i]))
    //      << " ";
    //    }
    //  }

    return actualLength;
}

void
transitHub::gypdatatransform(char* temp)
{
    unsigned short int tmp1[12];
    for (int i = 0; i < 12; i++) {
        tmp1[i] = temp[i];
    }

    unsigned short int tmp2[6];
    for (int i = 0; i < 6; i++) {
        tmp2[i] = ((tmp1[2 * i] << 8) & 0xff00) + (tmp1[2 * i + 1] & 0x00ff);
    }

    short int tmp[6];
    for (int i = 0; i < 6; i++) {
        if (tmp2[i] > 0xb000) {
            tmp[i] = (tmp2[i] | 0xc000);
        } else {
            tmp[i] = (tmp2[i] & 0x3fff);
        }
    }

    if (tmp[0] == 0 && tmp[1] == 0 && tmp[2] == 0 && tmp[3] == 0 && tmp[4] == 0 && tmp[5] == 0) // if error, just be equal to the former
    {
        ROS_ERROR("*************GYRO ERROR****************");
        return;
    }

    if (m_imu_version == "ADIS16365BMLZ") //  ADIS 16365BMLZ
    {
        gyroscope.GYPO[0] = tmp[0] * 0.05;
        gyroscope.GYPO[1] = tmp[1] * 0.05;
        gyroscope.GYPO[2] = tmp[2] * 0.05 + m_gyroscope_correct;
        gyroscope.ACCL[0] = tmp[3] * 0.00333 * 9.8;
        gyroscope.ACCL[1] = tmp[4] * 0.00333 * 9.8;
        gyroscope.ACCL[2] = tmp[5] * 0.00333 * 9.8;
    } else if (m_imu_version == "ADIS16355AMLZ") // ADIS 16355AMLZ default
    {
        gyroscope.GYPO[0] = tmp[0] * 0.07326;
        gyroscope.GYPO[1] = tmp[1] * 0.07326;
        gyroscope.GYPO[2] = tmp[2] * 0.07326 + m_gyroscope_correct;
        // std::cout << "tmp[0]:" << tmp[0] << std::endl;
        // std::cout << "tmp[1]:" << tmp[1] << std::endl;
        // std::cout << "tmp[2]:" << tmp[2] << std::endl;
        gyroscope.ACCL[0] = tmp[3] * 0.002522 * 9.8;
        gyroscope.ACCL[1] = tmp[4] * 0.002522 * 9.8;
        gyroscope.ACCL[2] = tmp[5] * 0.002522 * 9.8;
    } else if (m_imu_version == "ADIS16405BMLZ") // ADIS 16355AMLZ default
    {
        gyroscope.GYPO[0] = tmp[0] * 0.05;
        gyroscope.GYPO[1] = tmp[1] * 0.05;
        gyroscope.GYPO[2] = tmp[2] * 0.05 + m_gyroscope_correct;
        // std::cout << "tmp[0]:" << tmp[0] << std::endl;
        // std::cout << "tmp[1]:" << tmp[1] << std::endl;
        // std::cout << "tmp[2]:" << tmp[2] << std::endl;
        gyroscope.ACCL[0] = tmp[3] * 0.00333 * 9.8;
        gyroscope.ACCL[1] = tmp[4] * 0.00333 * 9.8;
        gyroscope.ACCL[2] = tmp[5] * 0.00333 * 9.8;
    } else if (m_imu_version == "MPU6000") {
        // gyro full range -1000dps~+1000dps
        // 16bits in 2'complement and 32.8 LSB/ °/s
        ROS_DEBUG("GYP 1");
        gyroscope.GYPO[0] = tmp[0] * 0.0304878;
        gyroscope.GYPO[1] = tmp[1] * 0.0304878;
        if (m_robot_version == 2012) {
            gyroscope.GYPO[2] = tmp[2] * 0.0304878 + m_gyroscope_correct;
        } else // "2012.5" and "2013"
        {
            gyroscope.GYPO[2] = -1.0 * (tmp[2] * 0.0304878);
        }
        // acc full range -8g~+8g
        // 16bits in 2'complement and  4096 LSB/g
        gyroscope.ACCL[0] = tmp[3] * 0.0002441 * 9.8;
        gyroscope.ACCL[1] = tmp[4] * 0.0002441 * 9.8;
        gyroscope.ACCL[2] = tmp[5] * 0.0002441 * 9.8;

    } else {
        ROS_ERROR("ERROR GYRO TYPE");
    }

    gyroscope.corresCycle = m_gaitCycle;

    timeval time_temp;


    if (m_gaitCycle <= 3) {
        gettimeofday(&time_temp, NULL);
        gyro_timeval[1] = 1000000 * time_temp.tv_sec + time_temp.tv_usec;
        gyro_timeval[1] /= 1000000;

        MOTION::sampletime = 0.02;
    } else // update the gyro_timeval[2] according to the real condition
    {
        gettimeofday(&time_temp, NULL);
        gyro_timeval[0] = gyro_timeval[1]; // set past
        gyro_timeval[1] = 1000000 * time_temp.tv_sec + time_temp.tv_usec;
        gyro_timeval[1] /= 1000000; // get new value

        MOTION::sampletime = gyro_timeval[1] - gyro_timeval[0];
        if (MOTION::sampletime > 0.03) {
            MOTION::sampletime = 0.03;
        }
    }
    // ----canceled by yyj-----
    // gyroscope.GYPO[2] += -1.0 * m_gyroscope_correct;
// -------------
    gyroscope.GYPO_a[0] = gyroscope.GYPO[0] - m_status->getGdata().GYPO[0];
    // gyroscope.GYPO_X[1] = gyroscope.GYPO_X[0] - m_status->getGdata().GYPO_X[0];
    if (abs(gyroscope.GYPO_a[0]) > 500) {
        gyroscope.GYPO[0] = m_status->getGdata().GYPO[0];
    }
    auto cur_time = ros::Time::now();
    auto elapsed_time = (cur_time - t_time).toSec();
    if(elapsed_time > 1){
        m_status->resetGyro();
    }
    t_time = cur_time;
    m_status->setGdata(gyroscope);
    m_status->updateEularAngle();
    m_status->checkStableState();

    //  LOG(TRANSITHUB_DEBUG, "GYRO DEBUG: " << m_status->getEularAngle().m_x << "
    //  ");
    //  << m_status->getEularAngle().m_y << " " << gyroscope.GYPO[0]
    //  << " " << gyroscope.GYPO[1] << " " << gyroscope.GYPO[2] << " "
    //  << gyroscope.ACCL[0] << " " << gyroscope.ACCL[1] << " "
    //  << gyroscope.ACCL[2] << " " << gyroscope.GYPO_a[0] << " "
    //  << MOTION::sampletime << endl;
}

void
transitHub::doCheckTx(int id, bool pos, bool temp, bool vol, bool load)
{
    int data[2];
    data[0] = id;
    data[1] = (pos ? 0x01 : 0) | (temp ? 0x02 : 0) | (vol ? 0x04 : 0) | (load ? 0x08 : 0);
    if (data[0] != 0 && data[1] != 0) {
        transmit(data, PACKET_CHECK);
    } else {
        transmit(NULL, PACKET_PING);
    }
}

void
transitHub::readOptions()
{
    if (!m_nh->getParam("dmotion/hardware/serial_port_name", comName))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/serial_port_number", comNumber))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/serial_port_baudrate", baudRate))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/robot_version", m_robot_version))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/imu_version", m_imu_version))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/gyroscope_correct", m_gyroscope_correct))
        ROS_FATAL("getParam error");
    if (!m_nh->getParam("dmotion/hardware/imu_prepare_time", m_imu_prepare_time))
        ROS_FATAL("getParam error");
}

/*---------------------------------------------------------------------------------
 * Public Member Functions
 *-------------------------------------------------------------------------------*/
transitHub::transitHub(ros::NodeHandle* nh, RobotStatus* rs)
  : m_nh(nh)
  , m_status(rs)
  , m_bOpen(false)
  , m_ifd(-1)
  , m_gaitCycle(0)
{
    readOptions();
    portOpen();
    update_initdata();
}

void
transitHub::update_initdata()
{
    initdata = m_status->getMotorinit();
    m_status->imu_prepare_time = m_imu_prepare_time;
}

transitHub::~transitHub()
{
    portClose();
}

void
transitHub::doRx(char* recv, int failureCount)
{
    ROS_DEBUG("Start doRx");
    char recBuffer[200];
    char type;
    int count = failureCount; // 0
    while (failureCount == 0 ? 1 : count--) {
        /**
         * Step 1 Head
         **/
        receive(recBuffer, 1);
        if (recBuffer[0] != (char)(0xee)) {
            continue;
        }
        receive(recBuffer, 1);
        if (recBuffer[0] != (char)(0xee)) {
            continue;
        }

        /**
         * Step 2 type
         **/
        //    LOG(TRANSITHUB_DEBUG, "Step 2 type" );
        receive(recBuffer, 1);
        type = recBuffer[0];

        if ((type & 0x00) != 0) {
            ROS_ERROR("0x00");
        }
        /**
         * Step 3 gyro
         **/
        if ((type & 0x10) != 0) {
            ROS_DEBUG("Step 3 gyro");
            char checksum = 0;
            receive(recBuffer, 13);
            for (int i = 0; i < 13; i++) {
                checksum += recBuffer[i];
                //        printf("%02x ", recBuffer[i] & 0xff);
            }
            //      cout << endl;
            if ((checksum & 0xff) != 0xff) {
                ROS_ERROR("doRx gyro checksum error!");
                continue;
            }
            gypdatatransform(recBuffer);
        }
        /**
         * Step 4 compass
         **/
        if ((type & 0x40) != 0) {
            ROS_DEBUG("Step 4 compass");
            char checksum = 0;
            int len = 9;
            receive(recBuffer, len);
            //      cout << endl;
            for (int i = 0; i < len; i++) {
                checksum += recBuffer[i];
                //        printf("%02x ", recBuffer[i] & 0xff);
            }
            //      cout << endl;
            if ((checksum & 0xff) != 0xff) {
                ROS_ERROR("Compass checksum error");
                continue;
            } else {
                short int x = (recBuffer[0] << 8) + (recBuffer[1] & 0xff);
                short int y = (recBuffer[2] << 8) + (recBuffer[3] & 0xff);
                short int z = (recBuffer[4] << 8) + (recBuffer[5] & 0xff);
                short int temperature = (recBuffer[6] << 8) + (recBuffer[7] & 0xff);

                //        if (x > 60000) {
                //          x -= 65536;
                //        }
                //        if (y > 60000) {
                //          y -= 65536;
                //        }
                //        if (z > 60000) {
                //          z -= 65536;
                //        }
                //        if (temperature > 60000) {
                //          temperature -= 65536;
                //        }

                compass.x = x;
                compass.y = y;
                compass.z = z;
                compass.temperature = temperature;
                compass.corresCycle = m_gaitCycle;
                m_status->setCompassData(compass);
                ROS_DEBUG("Compass %ld %ld %ld %lf", compass.x, compass.y, compass.z, compass.temperature);
            }
        }

        /**
         * Step 5 data request + state feedback
         **/
        ROS_DEBUG("step 5 data request and state feedback");
        if (recv != NULL) {
            receive(recBuffer, 2);
            recv[0] = recBuffer[0];
            recv[1] = recBuffer[1];
        }
        break;
    }
    // todo, check if dev exist
    ROS_DEBUG("Exit doRx");
}

void
transitHub::doLoopTx(int* body, int* camera)
{
    ROS_DEBUG("start doLoopTx");
    /* for storing the transmitted data */
    int store_data[MOTORNUM];
    /* for storing the received data */
    char recv[3];

    for (int i = 0; i < MOTORNUM; i++) {
        if (i < MOTORNUM - 2)
            store_data[i] = body[i];
        else if (i >= MOTORNUM - 2)
            store_data[i] = camera[i - (MOTORNUM - 2)];

        //    LOG(TRANSITHUB_DEBUG, __func__ << m_gaitCycle << " " << store_data[i]
        //    << " ");
    }
    transmit(store_data);

    doRx(recv);
    //  while (true) {
    //    doRx(recv);
    //    /* if lost, redo */
    //    if (recv[1] == (char) NONE) {
    //      LOG(ERROR, "do loopTx lost" );
    //      transmit(store_data);
    //      continue;
    //    }
    //
    //    if (recv[1] == (char) FAILURE)  // if error, redo
    //    {
    //      LOG(ERROR, "do loopTx error " );
    //      transmit(store_data);
    //      continue;
    //    }
    //
    //    while (recv[0] == (char) 'O')  // when the lower control board doesn't
    //      // require for data, just ping
    //    {
    //      LOG(ERROR, "doLoopTx just ping" );
    //      doCheckTx();
    //      doRx(recv);
    //    }
    //    break;
    //  }
    m_gaitCycle++;
}
