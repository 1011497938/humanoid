#pragma once
#include "item.hpp"

namespace dviz {

class Obstacles : public Item
{
  public:
    Obstacles(bool loc, int id, QGraphicsItem* parent = 0);

    QRectF boundingRect() const;
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
    void drawObstacle(QPainter* painter, qreal x, qreal y);

  private:
    int m_id;
    bool m_loc;
};
}
