#include "circle.hpp"
#include "dconfig/dconstant.hpp"
namespace dviz {

const float R = dconstant::geometry::centerCircleDiameter;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Circle::Circle(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(m_id);
}


void Circle::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    if(m_model->getLocSeeCircle()) {
        auto circlePos = m_model->getLocCircle();
        auto g = getGlobalPosition(m_model->getLocRobotPos(), circlePos);
        setPos(g);
        auto c = QColor(255, 255, 0, 80);
        painter->setPen(QPen(c, 5));
        painter->drawEllipse(rect);

        painter->setPen(Qt::black);
        drawText(0, 0, QString("Circle %1").arg(m_id));
    }
}

}
