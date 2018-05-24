#include "ball.hpp"
#include "dconfig/dconstant.hpp"
#include <QTimer>

namespace dviz {
const float ballR = dconstant::ballSize::diameter;
const QRectF ballRect = QRectF(-ballR / 2, -ballR / 2, ballR, ballR);

Ball::Ball(bool loc, int id, QGraphicsItem* parent)
  : m_id(id)
  , m_loc(loc)
  , Item(parent)
{
    if (!m_loc) {
        this->setFlag(QGraphicsItem::ItemIsMovable, true);
    } else {
        m_model = Model::getInstance(id);
    }
}

QRectF
Ball::boundingRect() const
{
    return ballRect;
}

void
Ball::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    if (!m_loc) {
        Model::setSimBallPos(x(), y());
    }

    if (m_loc) {
        auto ballField = m_model->getLocBallFieldPos();
        auto robotPos = m_model->getLocRobotPos();
        auto ballGlobal = getGlobalPosition(robotPos, ballField);
        setPos(ballGlobal);

        if (!m_model->getLocSeeball()) {
            return;
        }
    }

    auto c = QColor(255, 167, 0);
    if (m_loc)
        c = QColor(255, 167, 0, 100);

    painter->setPen(c);
    painter->setBrush(c);
    painter->drawEllipse(ballRect);

    painter->setPen(Qt::black);
    if (m_loc)
        drawText(10, 0, QString("%1 loc").arg(m_id));
    else
        drawText(10, 0, QString("%1").arg(m_id));
}
}
