#pragma once
#include "item.hpp"

namespace dviz {

class Robot : public Item {
public:
    Robot(int robot_id, bool loc, QGraphicsItem* parent = 0);
    QRectF boundingRect() const;
    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void locPaint(QPainter* painter);
    void realPaint(QPainter* painter);

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
private:
    int m_id;
    bool isLocrobot = false;
    QColor m_color;

    bool m_selected = false;
};

}
