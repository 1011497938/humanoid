#pragma once
#include <QVector3D>
#include <QMatrix>
#include <QDebug>
#include <cmath>
#include <QPainter>
#include <geometry_msgs/Vector3.h>

namespace dviz {
static const int NUM_ROBOT = 6;

inline void drawCircle(QPainter* painter) {
    painter->setPen(Qt::yellow);
    painter->setBrush(Qt::yellow);
    painter->drawEllipse(QRectF(0, 0, 100, 100));
}

enum MODE {
    MONITOR,
    SIMULATOR,
};


extern QMatrix flipY;

template <typename T>
inline T DegreeToRadian(T angle) {
    return angle / 180.f * M_PI;
}

template <typename T>
inline T RadianToDegree(T angle) {
    return angle / M_PI * 180.f;
}

template <typename T>
inline void CorrectAngleDegree180(T& deg) {
    T x_mod = std::fmod(deg + 180, 360);
    if(x_mod < 0) {
        x_mod += 360;
    }
    deg =  x_mod - 180;
}

template <typename T>
inline T GetSlope(T x1, T y1, T x2, T y2) {
    auto res = std::atan2(y2 - y1, x2 - x1);
//    qDebug() << x1 << y1 << x2 << y2 << y2 - y1 << x2 - x1 <<  RadianToDegree(res);
    return RadianToDegree(res);
}


template <typename T>
inline float GetSlopeP(T p1, T p2) {
    return GetSlope(p1.x(), p1.y(), p2.x(), p2.y());
}

template <typename T>
inline void rotateVector(T& vec, float degree) {
    auto x = vec.x();
    auto y = vec.y();
    auto r = DegreeToRadian(degree);

    auto sin_ = std::sin(r);
    auto cos_ = std::cos(r);

    vec.setX(x * cos_ - y * sin_);
    vec.setY(x * sin_ + y * cos_);
}


inline QVector3D getGlobalPosition(QVector3D robotPos, QVector3D pos) {
    auto rx = robotPos.x();
    auto ry = robotPos.y();
    auto rz = robotPos.z();

    rotateVector(pos, rz);

    auto x = pos.x();
    auto y = pos.y();
    auto z = pos.z();

    QVector3D res;
    res.setX(x + rx);
    res.setY(y + ry);
    res.setZ(z + rz);

    return res;
}

inline QPointF getGlobalPosition(QVector3D robotPos, QPointF pos) {
    auto v = getGlobalPosition(robotPos, QVector3D(pos.x(), pos.y(), 0));
    return QPointF(v.x(), v.y());
}

inline QVector3D getGlobalPosition(QVector3D robotPos, geometry_msgs::Vector3 pos) {
    auto v = getGlobalPosition(robotPos, QVector3D(pos.x, pos.y, pos.z));
    return QVector3D(v.x(), v.y(), v.z());
}

inline QVector3D getFieldPosition(QVector3D robotPos, QVector3D pos) {
    auto rx = robotPos.x();
    auto ry = robotPos.y();
    auto rz = robotPos.z();

    auto x = pos.x();
    auto y = pos.y();
    auto z = pos.z();

    QVector3D res = pos;
    res.setX(x - rx);
    res.setY(y - ry);
    res.setZ(z - rz);


    rotateVector(res, -rz);
    return res;
}

inline QPointF getFieldPosition(QVector3D robotPos, QPointF pos) {
    auto v = getFieldPosition(robotPos, QVector3D(pos.x(), pos.y(), 0));
    return QPointF(v.x(), v.y());
}

inline QPointF getFieldPosition(QVector3D robotPos, qreal x, qreal y) {
   return getFieldPosition(robotPos, QPointF(x, y));
}

inline QVector3D Vector3ToQVector3D(geometry_msgs::Vector3 in) {
    QVector3D res;
    res.setX(in.x);
    res.setY(in.y);
    res.setZ(in.z);
    return res;
}

inline QPointF Vector3ToQPointF(geometry_msgs::Vector3 in) {
    QPointF res;
    res.setX(in.x);
    res.setY(in.y);
    return res;
}

inline geometry_msgs::Vector3 QVector3DToVector3(QVector3D in) {
    geometry_msgs::Vector3 res;
    res.x = in.x();
    res.y = in.y();
    res.z = in.z();
    return res;
}

inline geometry_msgs::Vector3 QPointFToVector3(QPointF in) {
    geometry_msgs::Vector3 res;
    res.x = in.x();
    res.y = in.y();
    return res;
}

template <typename T>
double length(T v) {
   return sqrt(v.x() * v.x() + v.y() * v.y());
}

}
