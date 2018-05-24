#include "world.hpp"
#include "dconfig/dconstant.hpp"
namespace dviz {

World::World()
{
    this->setFlag(QGraphicsItem::ItemIsMovable, true);
}

QRectF World::boundingRect() const
{
    return QRectF(-100, -100, 200, 200);
}

void World::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    // x axis
    painter->setPen(QPen(Qt::red, 5));
    painter->drawLine(0, 0, 100, 0);

    // y axis
    painter->setPen(QPen(Qt::blue, 5));
    painter->drawLine(0, 0, 0, 100);

    this->drawText(100, 0, QString("(x: %1, y: %2 r: %3)").arg(this->x()).arg(this->y()).arg(this->angle()));

    // border
    if(false) {
        int w = dconstant::geometry::wholeWidth;
        int h = dconstant::geometry::wholeHeight;

        painter->setPen(QPen(Qt::red, 5));
        painter->drawRect( -w / 2, -h / 2, w, h);
    }
}

}
