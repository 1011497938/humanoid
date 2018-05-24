#pragma once
#include "dmonitor/baseObject.hpp"
#include "dmonitor/ball.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dmonitor/viewRange.hpp"
#include "dmsgs/VisionInfo.h"
#include "dmsgs/MotionInfo.h"
#include "dvision/amcl/map.hpp"
#include <QTime>
#include <opencv2/opencv.hpp>
using namespace cv;

namespace dmonitor {

class Robot : public BaseObject {
    Q_OBJECT
    Q_PROPERTY(QString address READ address WRITE setAddress)
    Q_PROPERTY(Ball* ball READ ball WRITE setBall)
    Q_PROPERTY(Ball* simBall READ simBall WRITE setSimBall)
    Q_PROPERTY(ViewRange* viewRange READ viewRange WRITE setViewRange NOTIFY viewRangeChanged)
    Q_PROPERTY(bool online READ online WRITE setOnline NOTIFY onlineChanged)

public:
    Robot(QQuickItem* parent = 0);
    ~Robot();

    // OVERRIDE FUNCTIONS
    void simModeUpdate() override;
    void monitorModeUpdate() override;
    void drawMyself(QPainter* painter) override;
    void drawLines(QPainter *painter);
    Q_INVOKABLE void init() override;

    // qml read
    QString address() const;
    Ball* ball() const;

    void drawCircle(QPainter *painter);
    void drawParticles(QPainter* painter);
    void drawParticle(QPainter* painter, float x, float y, float t, float weight);

    bool online() const;
    void drawView(QPainter *painter);
    Q_INVOKABLE void reset();
    void onRecvMotion(dmsgs::MotionInfo &msg);
    bool isOnline();
    Point3d realPos();
    void setPos(Point3d p);
    Ball* simBall() const;
    ViewRange *viewRange() const;

    std::vector<cv::Point2f> getWhitePointsInViewRange();

    bool inView(float, float);
    void drawRobot(QPainter *painter, bool sim);
public slots:
    void setAddress(QString address);
    void onRecv(dmsgs::VisionInfo& msg);
    void setBall(Ball* ball);
    void setOnline(bool online);
    void setSimBall(Ball* simBall);
    void setViewRange(ViewRange* viewRange);

signals:
    void onlineChanged(bool online);
    void viewRangeChanged(ViewRange* viewRange);

private:
    dtransmit::DTransmit* m_transmitter;
    dmsgs::VisionInfo m_simVisionInfo;
    dmsgs::VisionInfo m_monVisionInfo;
    dvision::Map m_map;
    dmsgs::MotionInfo m_motionInfo;

    QTime m_lastRecvTime;

    Ball* m_ball = nullptr;
    Ball* m_simBall = nullptr;
    ViewRange* m_viewRange = nullptr;

    QString m_address;

    QPointF m_realPos;
    QPointF m_locPos;
    QPointF m_ballPos;
    double m_heading;
    int m_triangleBBoxWidth = 40;
    int m_triangleBBoxHeight = 40;
    QColor m_color = QColor(255, 167, 0);
    QColor m_colorLight = QColor(255, 167, 0);
    const int MAX_UNSEEN_SEC = 15;

    bool m_online = false;
};

} // namespace dmonitor
