#include "robot.hpp"

namespace dviz {
Robot::Robot(int robot_id, bool loc, QGraphicsItem* parent)
    : Item(parent), m_id(robot_id), isLocrobot(loc)
{
    m_model = Model::getInstance(robot_id);
    if(!loc) {
        this->setFlag(QGraphicsItem::ItemIsMovable, true);
    }
}

QRectF Robot::boundingRect() const
{
    int r = 15;
    return QRectF(-r, -r, r*2, r*2);
}

static const std::vector<QPointF> points {
    QPointF(15, 0),
    QPointF(-10, -10),
    QPointF(-10, 10)
};


/* setPos for locrobot, locball
 *
 */

void Robot::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    if(isLocrobot)
        this->locPaint(painter);
    else
        this->realPaint(painter);
//    qDebug() << "robot paint";

}

void Robot::locPaint(QPainter *painter)
{
    QVector3D locPos = m_model->getLocRobotPos();
    setPos(locPos.x(), locPos.y());
    setRotation(locPos.z());


    auto c = QColor(255, 167, 0, 100);

    QPen pen(c, 0);

    painter->setPen(pen);
    painter->setBrush(c);
    painter->drawPolygon(points.data(), points.size());

    painter->setPen(Qt::black);

    drawText(0, 0, QString("%1 (%2, %3, %4)").arg(m_id).arg(x()).arg(y()).arg(this->m_angle));
}

void Robot::realPaint(QPainter *painter)
{
    if(!m_selected) {
        auto pos = m_model->getSimRobotPos();
        setPos(pos);
    }

    QColor c(255, 167, 0);
    QPen pen(c, 0);

    painter->setPen(pen);
    painter->setBrush(c);
    painter->drawPolygon(points.data(), points.size());

    painter->setPen(Qt::black);
    drawText(0, 0, QString("%1").arg(m_id));
}

void Robot::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    m_selected = true;
    Item::mousePressEvent(event);
}

void Robot::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    m_selected = false;
    m_model->setSimRobotPos(pos());
    Item::mouseReleaseEvent(event);
}

void Robot::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    m_model->setSimRobotPos(pos());
    Item::mouseMoveEvent(event);
}
}
