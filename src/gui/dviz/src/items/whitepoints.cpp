#include "whitepoints.hpp"
#include "item.hpp"

namespace dviz {

WhitePoints::WhitePoints(int id, QGraphicsItem *parent)
    : m_id(id), Item(parent)
{
    m_model = Model::getInstance(id);
}

void WhitePoints::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    auto& ps = m_model->getLocWhitePoints();
    painter->setPen(QPen(Qt::green, 3));
    auto locRobotPos = m_model->getLocRobotPos();
    for_each(ps.begin(), ps.end(), [&](QPointF& p){
        auto gp = getGlobalPosition(locRobotPos, p);
        painter->drawPoint(gp);
    });
}

}
