#include "goals.hpp"
#include "dconfig/dconstant.hpp"
namespace dviz {

const float R = dconstant::geometry::centerCircleDiameter;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Goals::Goals(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(m_id);
}


void Goals::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto goals = m_model->getFieldLocGoals();
    auto robotPos = m_model->getLocRobotPos();
    for_each(goals.begin(), goals.end(), [&](geometry_msgs::Vector3& p){
        auto g = getGlobalPosition(robotPos, p);
        this->drawGoal(painter, g.x(), g.y());
    });
}

void Goals::drawGoal(QPainter *painter, qreal x, qreal y)
{
    painter->setPen(QPen(Qt::red, 5));
    painter->setBrush(Qt::red);
    QRectF rect(x - 5, y - 5, 10, 10);
    painter->drawEllipse(rect);
    drawText(x + 5, y, QString("G %1").arg(m_id));
}

}
