#include "model.hpp"
#include "dconfig/dconstant.hpp"
#include "dtransmit/dtransmit.hpp"
#include "ros/ros.h"
#include <QDebug>
#include <QTimer>
#include <functional>
//#define DEBUG
using dtransmit::DTransmit;
using namespace dconstant::network;
using namespace std;

//const string Localhost = "127.0.0.1";
//const string Lan = "192.168.255.255";
const int MAX_UNSEEN_SEC = 10;

namespace dviz {

array<Model*, NUM_ROBOT + 1> Model::instances = { NULL };
MODE Model::mode = MODE::MONITOR;
bool Model::showParticles = false;
bool Model::showViewRange = false;
QPointF Model::m_simBallPos = QPointF(200, 200);
QPointF Model::m_simObstaclePos = QPointF(300, 300);

Model::Model(int id, QObject* parent)
  : QObject(parent)
  , m_id(id)
{
#ifdef DEBUG
    if (m_id == 1) {
        auto t = new QTimer(this);

        connect(t, &QTimer::timeout, [this]() {
            emit robotConnectionStatusChanged(m_id, true);
            auto z = m_locRobotPos.z();
            m_locRobotPos.setZ(z + 1);

            auto updatedPos = getGlobalPosition(m_realRobotPos, m_motionDelta);
            m_realRobotPos = updatedPos;
        });

        t->start(1000 / 30.f);

        m_locParticles.resize(1);
        auto& p = m_locParticles[0];
        p.pose.x = -100;
        p.pose.y = -100;
        p.pose.z = 45;
    }
#endif
    m_map.Init();

    m_lastRecvTime = QTime::currentTime().addSecs(-MAX_UNSEEN_SEC * 2);

    m_transmitter = new DTransmit("255.255.255.255");
    m_transmitter->addRosRecv<dmsgs::VisionInfo>(robotBroadcastAddressBase + m_id, std::bind(&Model::onRecvVisionInfo, this, std::placeholders::_1));

    m_transmitter->addRosRecv<dmsgs::MotionInfo>(robotMotionBase + m_id, std::bind(&Model::onRecvMotionInfo, this, std::placeholders::_1));

    m_transmitter->startService();

    // check connection
    {
        QTimer* t = new QTimer(this);
        connect(t, &QTimer::timeout, [this]() {
            auto now = QTime::currentTime();
            auto last = m_lastRecvTime.secsTo(now);
            bool status;
            if (last > MAX_UNSEEN_SEC) {
                status = false;
            } else {
                status = true;
            }

            if (status != m_connected) {
                m_connected = status;
                emit robotConnectionStatusChanged(m_id, status);
            }
        });
        t->start(1000);
    }

    // send sim info at 30fps
    {
        QTimer* t = new QTimer(this);
        connect(t, &QTimer::timeout, [this]() {
            if (mode == SIMULATOR)
                this->sendSimVisionInfo();
        });
        t->start(1000 / 30.f);
    }
}

Model::~Model()
{
}

bool
Model::isEnabled()
{
    return m_enabled;
}

bool
Model::isConnected()
{
    return m_connected;
}

// Setter & Getter

QVector3D
Model::getLocRobotPos()
{
    Lock l(m_lock);
    return m_locRobotPos;
}

QVector3D
Model::getSimRobotPos()
{
    Lock l(m_lock);
    //    qDebug() << "return " << m_simRobotPos;
    return m_simRobotPos;
}

QVector3D
Model::getMotionDelta()
{
    Lock l(m_lock);
    return m_motionDelta;
}

QVector3D
Model::getDest()
{
    Lock l(m_lock);
    return m_dest;
}

QVector3D
Model::getFinalDest()
{
    Lock l(m_lock);
    return m_final_dest;
}

bool
Model::getLocSeeball()
{
    Lock l(m_lock);
    return m_locSeeball;
}

QPointF
Model::getLocBallFieldPos()
{
    Lock l(m_lock);
    return m_locBallPosField;
}

bool
Model::getLocSeeCircle()
{
    Lock l(m_lock);
    return m_locSeeCircle;
}

QPointF
Model::getLocCircle()
{
    Lock l(m_lock);
    return m_locCircleField;
}

std::vector<QPointF>&
Model::getLocFieldViewRange()
{
    Lock l(m_lock);
    return m_locFieldViewRange;
}

std::vector<QPointF>&
Model::getLocWhitePoints()
{
    Lock l(m_lock);
    return m_locWhitePoints;
}

std::vector<dmsgs::ParticleMsg>&
Model::getParticles()
{
    Lock l(m_lock);
    return m_locParticles;
}

std::vector<dmsgs::Line>&
Model::getLocWhiteLines()
{
    Lock l(m_lock);
    return m_locFieldLines;
}

std::vector<geometry_msgs::Vector3>&
Model::getFieldLocGoals()
{
    Lock l(m_lock);
    return m_locFieldGoals;
}

std::vector<geometry_msgs::Vector3>&
Model::getFieldLocObstacles()
{
    Lock l(m_lock);
    return m_locFieldObstacles;
}

dvision::Map&
Model::getMap()
{
    Lock l(m_lock);
    return m_map;
}

std::vector<geometry_msgs::Vector3>&
Model::getSimWhitepoints()
{
    Lock l(m_lock);
    return m_simWhitePoints;
}

std::vector<geometry_msgs::Vector3>&
Model::getSimGoalPosts()
{
    Lock l(m_lock);
    return m_simGoalPosts;
}

std::vector<geometry_msgs::Vector3>&
Model::getSimObstacles()
{
    Lock l(m_lock);
    return m_simObstacles;
}

void
Model::setSimRobotPos(QVector3D pos)
{
    Lock l(m_lock);
    m_simRobotPos = pos;
}

void
Model::setSeeSimball(bool see)
{
    Lock l(m_lock);
    m_seeSimBall = see;
}

void
Model::setSeeSimObstacle(bool see)
{
    Lock l(m_lock);
    m_seeSimObstacle = see;
}

// Setter & Getter end

MODE
Model::getMode()
{
    return mode;
}

void
Model::setMode(MODE m)
{
    mode = m;
}

QPointF
Model::getSimBallPos()
{
    return m_simBallPos;
}

void
Model::setSimBallPos(qreal x, qreal y)
{
    m_simBallPos.setX(x);
    m_simBallPos.setY(y);
}

QPointF
Model::getSimObstaclePos()
{
    return m_simObstaclePos;
}

void
Model::setSimObstaclePos(qreal x, qreal y)
{
    m_simObstaclePos.setX(x);
    m_simObstaclePos.setY(y);
}

void
Model::setSeeSimCircle(bool see)
{
    Lock l(m_lock);
    m_seeSimCircle = see;
}

void
getVec(vector<QPointF>& res, std::vector<geometry_msgs::Vector3>& points)
{
    res.resize(points.size());
    for (uint32_t i = 0; i < res.size(); ++i) {
        res[i].setX(points[i].x);
        res[i].setY(points[i].y);
    }
}

void
Model::onRecvVisionInfo(dmsgs::VisionInfo& msg)
{
    Lock l(m_lock);
    m_lastRecvTime = QTime::currentTime();
    getVec(m_locFieldViewRange, msg.viewRange);
    getVec(m_locWhitePoints, msg.locFieldWhitePoints);
    m_locParticles = msg.particles;
    m_locSeeball = msg.see_ball;

    m_locSeeCircle = msg.see_circle;

    m_locRobotPos = Vector3ToQVector3D(msg.robot_pos);
    m_locBallPosField = Vector3ToQPointF(msg.ball_field);
    m_locCircleField = Vector3ToQPointF(msg.circle_field);
    m_locFieldGoals = msg.goals_field;
    m_locFieldObstacles = msg.obstacles_field;
    m_locFieldLines = msg.lines_field;

    m_dest = Vector3ToQVector3D(msg.behaviorInfo.dest);
    m_final_dest = Vector3ToQVector3D(msg.behaviorInfo.final_dest);
}

void
Model::onRecvMotionInfo(dmsgs::MotionInfo& msg)
{
    Lock l(m_lock);
    if (mode == MONITOR)
        return;
    m_lastRecvTime = QTime::currentTime();

    {
        auto tmpDelta = Vector3ToQVector3D(msg.deltaData);
        auto d = tmpDelta - m_prevDelta;

        auto dx = d.x();
        auto dy = d.y();
        auto dt = d.z();
        auto t = m_prevDelta.z() / 180.0 * M_PI;

        auto ddx = dx * cos(-t) - dy * sin(-t);
        auto ddy = dx * sin(-t) + dy * cos(-t);

        m_prevDelta = tmpDelta;

        m_motionDelta.setX(ddx);
        m_motionDelta.setY(ddy);
        m_motionDelta.setZ(dt);
    }

    // update robot position
    auto updatedPos = getGlobalPosition(m_simRobotPos, m_motionDelta);
    m_simRobotPos = updatedPos;
}

void
Model::sendSimVisionInfo()
{
    Lock l(m_lock);
    if (!m_connected)
        return;
    dmsgs::VisionInfo info;
    info.see_ball = m_seeSimBall;
    info.ball_field = QPointFToVector3(getFieldPosition(m_simRobotPos, m_simBallPos));

    info.see_circle = m_seeSimCircle;
    info.circle_field = QPointFToVector3(getFieldPosition(m_simRobotPos, m_simCircleCenter));

    info.simFieldWhitePoints = m_simWhitePoints;
    info.simYaw = m_simRobotPos.z();

    info.goals_field = m_simGoalPosts;

    info.see_obstacle = m_seeSimObstacle;
    info.obstacles_field = m_simObstacles;

    m_transmitter->sendRos<dmsgs::VisionInfo>(monitorBroadcastAddressBase + m_id, info);
}

Model*
Model::getInstance(int id)
{
    if (id > NUM_ROBOT)
        return NULL;

    if (!instances[id]) {
        instances[id] = new Model(id);
    }

    return instances[id];
}

void
Model::setEnable(bool enabled)
{
    m_enabled = enabled;
}

void
Model::onSimRobotPosChanged(qreal x, qreal y, qreal angle)
{
    Lock l(m_lock);
    m_simRobotPos.setX(x);
    m_simRobotPos.setY(y);
    m_simRobotPos.setZ(angle);
}
}
