#pragma once
#include "item.hpp"
namespace dviz {
class Dest : public Item {
public:
    Dest(int id, QGraphicsItem* parent = 0);

    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void drawP(QPainter *painter, qreal x, qreal y, qreal angle);
private:
    int m_id;
};
}
