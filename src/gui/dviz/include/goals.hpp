#pragma once
#include "item.hpp"

namespace dviz {

class Goals : public Item {
public:
    Goals(int id, QGraphicsItem* parent = 0);

    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void drawGoal(QPainter* painter, qreal x, qreal y);

private:
    int m_id;

};

}
