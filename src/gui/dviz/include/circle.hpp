#pragma once
#include "item.hpp"

namespace dviz {

class Circle : public Item {
public:
    Circle(int id, QGraphicsItem* parent = 0);

    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    int m_id;
private:

};

}
