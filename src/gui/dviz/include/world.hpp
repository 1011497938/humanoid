#pragma once
#include "item.hpp"

namespace dviz {

class World : public Item {
public:
    World();
    QRectF boundingRect() const;
    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};
}
