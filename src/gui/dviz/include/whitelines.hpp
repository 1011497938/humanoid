#pragma once

#include "item.hpp"

namespace dviz {

class WhiteLines : public Item {
public:
    WhiteLines(int id, QGraphicsItem* parent = 0);

    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
    int m_id;

};

}
