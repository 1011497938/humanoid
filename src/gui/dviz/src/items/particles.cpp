#include "particles.hpp"

namespace dviz {

Particles::Particles(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(m_id);
}

static const std::vector<QPointF> points {
    QPointF(7.5, 0),
    QPointF(-5, -5),
    QPointF(-5, 5)
};

void Particles::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto& ps = m_model->getParticles();
    for(auto& p : ps) {
        drawP(painter, p.pose.x, p.pose.y, p.pose.z, p.weight);
    }
}


void Particles::drawP(QPainter* painter, qreal x, qreal y, qreal angle, qreal weight) {
    painter->translate(x, y);
    painter->rotate(angle);

    QColor m_colorLight = QColor(0, 0, 0);
    QPen pen(m_colorLight, 0);
    painter->setPen(pen);
//    painter->setBrush(m_colorLight);
    painter->setBrush(m_colorLight.lighter(100 + (1 - weight) * 100));
    painter->drawPolygon(points.data(), points.size());

    // back
    painter->rotate(-angle);
    painter->translate(-x, -y);
}


}
