#include "dest.hpp"

namespace dviz {

Dest::Dest(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(m_id);
}

static const std::vector<QPointF> points {
    QPointF(7.5, 0),
    QPointF(-5, -5),
    QPointF(-5, 5)
};

void Dest::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto dest = m_model->getDest();
    drawP(painter, dest.x(), dest.y(), dest.z());

    auto final_dest = m_model->getFinalDest();
    drawP(painter, final_dest.x(), final_dest.y(), final_dest.z());
}

void Dest::drawP(QPainter* painter, qreal x, qreal y, qreal angle) {
    painter->translate(x, y);
    painter->rotate(angle);

    QColor m_colorLight = QColor(255, 220, 0, 100);
    QPen pen(m_colorLight, 0);
    painter->setPen(pen);
    painter->setBrush(m_colorLight);
    painter->drawPolygon(points.data(), points.size());

    // back
    painter->rotate(-angle);
    painter->translate(-x, -y);
}


}
