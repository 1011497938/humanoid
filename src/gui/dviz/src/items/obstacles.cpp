#include "obstacles.hpp"
#include "dconfig/dconstant.hpp"
namespace dviz {

const float ObstacleR = dconstant::ballSize::diameter;
const QRectF ObstacleRect = QRectF(-ObstacleR / 2, -ObstacleR / 2, ObstacleR, ObstacleR);

Obstacles::Obstacles(bool loc, int id, QGraphicsItem* parent)
  : m_id(id)
  , m_loc(loc)
  , Item(parent)
{
    if (!m_loc) {
        this->setFlag(QGraphicsItem::ItemIsMovable, true);
    } else {
        m_model = Model::getInstance(m_id);
    }
}

QRectF
Obstacles::boundingRect() const
{
    return ObstacleRect;
}

void
Obstacles::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    if (m_loc) {
        auto obstacles = m_model->getFieldLocObstacles();
        auto robotPos = m_model->getLocRobotPos();
        for_each(obstacles.begin(), obstacles.end(), [&](geometry_msgs::Vector3& p) {
            auto g = getGlobalPosition(robotPos, p);
            setPos(g);
            this->drawObstacle(painter, g.x(), g.y());
        });
    } else {
        Model::setSimObstaclePos(x(), y());
        this->drawObstacle(painter, x(), y());
    }
}

void
Obstacles::drawObstacle(QPainter* painter, qreal x, qreal y)
{
    auto color = QColor(8, 131, 111, 255);
    if (m_loc) {
        color = QColor(8, 131, 111, 100);
    }

    painter->setPen(color);
    painter->setBrush(color);
    // QRectF rect(x - 5, y - 5, 10, 10);
    painter->drawEllipse(ObstacleRect);
    // drawText(x + 5, y, QString("Obs %1").arg(m_id));
    if (m_loc)
        drawText(10, 0, QString("%1 obs").arg(m_id));
    else
        drawText(10, 0, QString("%1").arg(m_id));
}
}
