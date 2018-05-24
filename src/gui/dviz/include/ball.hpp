#pragma once
#include "item.hpp"
namespace dviz {
class Ball : public Item
{
  public:
    Ball(bool loc, int id = 0, QGraphicsItem* parent = 0);

    QRectF boundingRect() const;
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

    int m_id;
  public slots:
    void onCollision();

  private:
    bool m_loc;
};
}
