#include "dmonitor/ball.hpp"
#include <QPainter>

namespace dmonitor {
Ball::Ball(QQuickItem *parent) : BaseObject(parent) {
}

void Ball::init()
{
}

void Ball::drawMyself(QPainter* painter)
{
    if(isMonitor()) {
        auto imgPos = m_field->getOnImageCoordiante(m_realPos);
        setX(imgPos.x() - m_diameter / m_field->getScale() / 2);
        setY(imgPos.y() - m_diameter / m_field->getScale() / 2);
    }

    if(!m_field) {
        qDebug() << "field not set!";
        return;
    }

    double scale = m_field->getScale();
    setWidth(m_diameter / scale);
    setHeight(m_diameter / scale);
    QRectF rec(0, 0, width(), height());
    painter->setBrush(m_color);
    painter->setPen(QPen(m_color, 0));
    painter->drawEllipse(rec);
}



} // namespace dmonitor
