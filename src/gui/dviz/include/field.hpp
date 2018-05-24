#pragma once
#include "item.hpp"

namespace dviz {

class FieldItem : public Item {
public:
    FieldItem(QGraphicsItem* parent = 0);
    ~FieldItem();

    QRectF boundingRect() const;
    void myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

};

}
