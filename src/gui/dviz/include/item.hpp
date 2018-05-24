#pragma once
#include <QGraphicsItem>
#include <QtCore>
#include <QPainter>
#include <QDebug>
#include <QTimer>
#include <QVector3D>
#include "model.hpp"
#include "dconfig/dconstant.hpp"
#include "common.hpp"

namespace dviz {
class Model;
class Item : public QGraphicsItem {
public:
    Item(QGraphicsItem* parent = 0);
    virtual ~Item();

    QRectF boundingRect() const;

    qreal y() const;
    void setY(qreal y);
    void setPos(qreal x, qreal y);
    void setPos(QPointF);
    void setPos(QVector3D pos);
    QVector3D pos() const;
    qreal angle() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void flipPainterYAxis();
    virtual void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void drawText(qreal x, qreal y, QString str);
    void setRotation(qreal);
    void rotateDegree(qreal r);
    void rotateRadian(qreal r);

    void drawText(QPointF p, QString str);
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event);

    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);

    void setPainter(QPainter* p);
    Model* m_model;
    QPainter* m_painter;

    qreal m_angle = 0;
    bool m_isPressed = false;
    bool m_enableRotate = false;
    QPointF m_startPressPos;
    qreal m_startPressAngle = 0;
    qreal m_startPressAngle2 = 0;

};
}
