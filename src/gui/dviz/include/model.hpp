#pragma once
#include "common.hpp"
#include "dmsgs/MotionInfo.h"
#include "dmsgs/VisionInfo.h"

#include "dvision/amcl/map.hpp"
//#include "dtransmit/dtransmit.hpp"
#include "ros/ros.h"
//#include "dvision/frame.hpp"
#include <QDebug>
#include <QObject>
#include <QPointF>
#include <QTime>
#include <QVector3D>
#include <QtCore>
#include <array>
#include <mutex>
#include <vector>

namespace dtransmit {
class DTransmit;
}

namespace dviz {

class Model : public QObject
{
    typedef std::lock_guard<std::mutex> Lock;
    Q_OBJECT
  public:
    ~Model();
    bool isEnabled();
    bool isConnected();

    // API for data manipulating

    // Getter
    QVector3D getLocRobotPos();
    QVector3D getSimRobotPos();
    QVector3D getMotionDelta();
    QVector3D getDest();
    QVector3D getFinalDest();

    bool getLocSeeball();
    QPointF getLocBallFieldPos();

    bool getLocSeeCircle();
    QPointF getLocCircle();

    std::vector<QPointF>& getLocFieldViewRange(); // field position
    std::vector<QPointF>& getLocWhitePoints();    // field position
    std::vector<dmsgs::ParticleMsg>& getParticles();
    std::vector<dmsgs::Line>& getLocWhiteLines();
    std::vector<geometry_msgs::Vector3>& getFieldLocGoals();
    std::vector<geometry_msgs::Vector3>& getFieldLocObstacles();

    dvision::Map& getMap();

    std::vector<geometry_msgs::Vector3>& getSimWhitepoints();
    std::vector<geometry_msgs::Vector3>& getSimGoalPosts();
    std::vector<geometry_msgs::Vector3>& getSimObstacles();

    // Setter
    void setSimRobotPos(QVector3D pos);
    void setSeeSimball(bool see);
    void setSeeSimObstacle(bool see);

  private:
    QVector3D m_locRobotPos = { 100, 100, 45 };
    QVector3D m_prevDelta = { 0, 0, 0 };
    QVector3D m_motionDelta = { 0, 0, 0 };
    bool m_locSeeball = false;
    QPointF m_locBallPosField;
    bool m_locSeeCircle = false;
    QPointF m_locCircleField;
    std::vector<QPointF> m_locFieldViewRange;
    std::vector<dmsgs::ParticleMsg> m_locParticles;
    std::vector<geometry_msgs::Vector3> m_locFieldGoals;
    std::vector<geometry_msgs::Vector3> m_locFieldObstacles;
    std::vector<QPointF> m_locWhitePoints;
    std::vector<dmsgs::Line> m_locFieldLines;
    QPointF m_locLeftGoal;
    QPointF m_locRightGoal;
    QPointF m_locUnknownGoal;
    QVector3D m_dest;
    QVector3D m_final_dest;

    std::vector<geometry_msgs::Vector3> m_simWhitePoints;
    std::vector<geometry_msgs::Vector3> m_simGoalPosts;
    std::vector<geometry_msgs::Vector3> m_simObstacles;

    QVector3D m_simRobotPos;
    static QPointF m_simBallPos;
    static QPointF m_simObstaclePos;
    QPointF m_simCircleCenter = { 0, 0 };

  signals:
    void robotConnectionStatusChanged(int id, bool connected);

  public slots:
    void setEnable(bool enabled);
    void onSimRobotPosChanged(qreal x, qreal y, qreal angle);

    // static methods
  public:
    static Model* getInstance(int id);
    static MODE getMode();
    static void setMode(MODE m);
    static bool showViewRange;
    static bool showParticles;

    static QPointF getSimBallPos();
    static void setSimBallPos(qreal x, qreal y);
    static QPointF getSimObstaclePos();
    static void setSimObstaclePos(qreal x, qreal y);

    void setSeeSimCircle(bool see);

    std::vector<geometry_msgs::Vector3> getWhitePointsInViewRange();

  private:
    void onRecvVisionInfo(dmsgs::VisionInfo& msg);
    void onRecvMotionInfo(dmsgs::MotionInfo& msg);
    void sendSimVisionInfo();

  private:
    static MODE mode;

    int m_id;
    bool m_connected = false;
    bool m_enabled = false;
    bool m_seeSimBall = false;
    bool m_seeSimObstacle = false;
    bool m_seeSimCircle = false;
    int cycle_ = 0;
    dtransmit::DTransmit* m_transmitter;
    dvision::Map m_map;
    QTime m_lastRecvTime;
    std::mutex m_lock;

    Model(int id, QObject* parent = 0);
    static std::array<Model*, NUM_ROBOT + 1> instances;
};
}
