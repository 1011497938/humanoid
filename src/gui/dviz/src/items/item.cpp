#include "item.hpp"
#include "model.hpp"
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>

namespace dviz {

Item::Item(QGraphicsItem *parent)
    : QGraphicsItem(parent)
{
    this->setFlag(QGraphicsItem::ItemIsFocusable, true);
    setVisible(false);
}

Item::~Item()
{

}

QRectF Item::boundingRect() const
{
    return QRectF(0, 0, 0, 0);
}

qreal Item::y() const
{
   return -QGraphicsItem::y();

}

void Item::setY(qreal y)
{
    QGraphicsItem::setY(-y);
}

void Item::setPos(qreal x, qreal y)
{
   setX(x);
   setY(y);
}

void Item::setPos(QPointF p)
{
   setPos(p.x(), p.y());
}

void Item::setPos(QVector3D pos)
{
    setX(pos.x());
    setY(pos.y());
    setRotation(pos.z());
}

QVector3D Item::pos() const
{
    return QVector3D(x(), y(), m_angle);
}

qreal Item::angle() const
{
    return m_angle;
}

void Item::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    setPainter(painter);
    flipPainterYAxis();
    painter->rotate(m_angle);

    if(m_enableRotate) {
        auto c = QColor(240, 230, 140, 150);
        painter->setPen(QPen(c, 0));
        auto rect = this->boundingRect();
        painter->setBrush(c);
        painter->drawEllipse(rect);
    }

    this->myPaint(painter, option, widget);

    int w_2 = dconstant::geometry::wholeWidth / 2 - 20;
    int h_2 = dconstant::geometry::wholeHeight / 2 - 20;

    int x_ = x();
    int y_ = y();

    if(x_ < -w_2) {
        setX(-w_2);
    } else if(x_ > w_2) {
        setX(w_2);
    }

    if(y_ < -h_2) {
        setY(-h_2);
    } else if (y_ > h_2) {
        setY(h_2);
    }

}

void Item::flipPainterYAxis()
{
    if(!m_painter)
        return;

    qreal scale_ = scale();
//    QMatrix m;
//    m.scale(1, -1);
//    painter->setMatrix(m);

    auto m = m_painter->matrix();
    m.scale(scale_, -scale_);
    m_painter->setMatrix(m);
}

void Item::myPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{

}

void Item::drawText(QPointF p, QString str) {
    drawText(p.x(), p.y(), str);
}

void Item::drawText(qreal x, qreal y, QString str)
{
    m_painter->rotate(-m_angle);
    flipPainterYAxis();

    auto sin_ = std::sin(DegreeToRadian(m_angle));
    auto cos_ = std::cos(DegreeToRadian(m_angle));
    auto x_ = x * cos_ - y * sin_;
    auto y_ = x * sin_ + y * cos_;

    m_painter->drawText(x_, -y_, str);
    flipPainterYAxis();
    m_painter->rotate(m_angle);
}

void Item::setRotation(qreal r)
{
    m_angle =  r;
    CorrectAngleDegree180(m_angle);
}

void Item::rotateDegree(qreal r)
{
    m_angle += r;
    CorrectAngleDegree180(m_angle);
}

void Item::rotateRadian(qreal r)
{
    m_angle += RadianToDegree(r);
    CorrectAngleDegree180(m_angle);
}

void Item::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if(m_enableRotate) {
        m_isPressed = true;
        m_startPressAngle = m_angle;
        m_startPressAngle2 = GetSlope(x(), y(), event->pos().x(), -event->pos().y());
    }
    QGraphicsItem::mousePressEvent(event);
}

void Item::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    m_isPressed = false;
    QGraphicsItem::mouseReleaseEvent(event);
}

void Item::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(m_enableRotate) {
        auto t2 = GetSlope(x(), y(), event->pos().x(), -event->pos().y());
        m_angle = m_startPressAngle + t2 - m_startPressAngle2;
    } else {
        QGraphicsItem::mouseMoveEvent(event);
    }
}

void Item::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Shift) {
        m_enableRotate = true;
    }
}

void Item::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Shift) {
        m_enableRotate = false;
    }
}


void Item::setPainter(QPainter* p)
{
    m_painter = p;
}

}
