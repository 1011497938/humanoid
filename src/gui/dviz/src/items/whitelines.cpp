#include "whitelines.hpp"
namespace dviz {

WhiteLines::WhiteLines(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(id);
}


void WhiteLines::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto& lines = m_model->getLocWhiteLines();

    painter->setPen(QPen(Qt::white, 2));
    foreach(const auto& line, lines) {
        auto& p1 = line.endpoint1;
        auto& p2 = line.endpoint2;

        painter->drawLine(p1.x, p1.y, p2.x, p2.y);
    }
}

}
