#pragma once
#include "item.hpp"
namespace dviz {
class Particles : public Item {
public:
    Particles(int id, QGraphicsItem* parent = 0);

    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void drawP(QPainter *painter, qreal x, qreal y, qreal angle, qreal weight);
private:
    int m_id;
};
}
