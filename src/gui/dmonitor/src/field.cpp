#include "dmonitor/field.hpp"
#include <QDebug>
#include <QPainter>
namespace dmonitor {

Field::Field(QQuickItem *parent) : QQuickPaintedItem(parent)
{
}

void Field::init()
{
   //setWidth(m_imageWidth);
   //setHeight(m_imageHeight);
}

void Field::paint(QPainter *painter)
{
    m_scale = m_imageWidth / width();
    double h = width() / m_imageRatio;
    setHeight(h);
    drawField(painter);
}



void Field::drawField(QPainter *painter)
{
    // antialiasing
    painter->setRenderHint(QPainter::Antialiasing);
    // draw background
    painter->fillRect(0, 0, width(), height(), m_grassGreen);


    // draw border
    QPointF upleftCorner = getOnImageCoordiante(QPoint(-m_fieldLength / 2, m_fieldWidth / 2));
    QRectF border(upleftCorner, flipFromOrigin(upleftCorner));

    QPen pen(Qt::white, m_lineWidth / m_scale);
    painter->setPen(pen);
    painter->drawRect(border);

    // draw goal
    QPointF leftGoalUpleft = getOnImageCoordiante(QPointF(-m_fieldLength / 2 - m_goalDepth, m_goalWidth /2));
    QPointF leftGoalDownRight = getOnImageCoordiante(QPointF(-m_fieldLength / 2, -m_goalWidth / 2));
    QRectF leftGoal(leftGoalUpleft, leftGoalDownRight);
    painter->drawRect(leftGoal);
    painter->drawRect(flipFromOrigin(leftGoal));

    // draw goalArea
    QPointF leftGoalAreaUpLeft = getOnImageCoordiante(QPointF(-m_fieldLength / 2, m_goalAreaWidth / 2));
    QPointF leftGoalAreaDownRight = getOnImageCoordiante(QPointF(-m_fieldLength / 2 + m_goalAreaLength, -m_goalAreaWidth / 2));
    QRectF leftGoalArea(leftGoalAreaUpLeft, leftGoalAreaDownRight);
    painter->drawRect(leftGoalArea);
    painter->drawRect(flipFromOrigin(leftGoalArea));

    // draw center line
    QPointF centerLineUp = getOnImageCoordiante(QPointF(0, m_fieldWidth / 2 - 1));
    painter->drawLine(centerLineUp, flipFromOrigin(centerLineUp));

    // draw center cirlce
    QPointF circleUpleft = getOnImageCoordiante(QPointF(m_centerCircleDiameter / 2, -m_centerCircleDiameter / 2));
    QRectF centerCircle(circleUpleft, flipFromOrigin(circleUpleft));
    painter->drawEllipse(centerCircle);
    auto center = getOnImageCoordiante(QPointF(0, 0));
    QPointF x(center.x() - m_lineWidth / m_scale / 2, center.y());
    QPointF y(center.x() + m_lineWidth / m_scale / 2, center.y());
    painter->drawLine(x, y);

    // draw penalty mark
    QPointF leftPenalty = getOnImageCoordiante(QPointF(-m_fieldLength / 2 + m_penaltyMarkDistance, 0));
    drawCross(painter, leftPenalty);
    drawCross(painter, flipFromOrigin(leftPenalty));
}


void Field::drawCross(QPainter* painter, QPointF img) {
    // draw horizontal line
    {
        QPointF x(img.x() - m_lineWidth / m_scale, img.y());
        QPointF y(img.x() + m_lineWidth / m_scale, img.y());
        painter->drawRect(QRectF(x, y));
    }

    // draw vertical line
    {
        QPointF x(img.x(), img.y() - m_lineWidth / m_scale);
        QPointF y(img.x(), img.y() + m_lineWidth / m_scale);
        painter->drawRect(QRectF(x, y));
    }
}

void Field::dragEnterEvent(QDragEnterEvent *ev)
{
    qDebug() << "drag enter event";
}

void Field::mouseMoveEvent(QMouseEvent *event)
{
   qDebug() << "mouseMoveEvent";
}

QPointF Field::getOnImageCoordiante(QPointF real)
{
    QPointF res;
    res.setX(width() / 2 + real.x() / m_scale);
    res.setY(height() / 2 - real.y() / m_scale);
    return res;
}

QPointF Field::getOnImageCoordiante(cv::Point2f real)
{
    return getOnImageCoordiante(getQPoint(real));
}

QPointF Field::getOnRealCoordinate(QPointF img) {
    QPointF res;
    res.setX((img.x() - width() / 2) * m_scale);
    res.setY((-img.y() + height() /2) * m_scale);
    return res;
}

QPointF Field::getOnRealCoordinate(Point2f img)
{
    return getOnRealCoordinate(getQPoint(img));
}

QPointF Field::flipFromOrigin(QPointF p)
{
    auto real = getOnRealCoordinate(p);
    real.setX(-real.x());
    real.setY(-real.y());
    return getOnImageCoordiante(real);
}

QRectF Field::flipFromOrigin(QRectF rect)
{
   auto topLeft = rect.topLeft();
   auto bottomRight = rect.bottomRight();

   auto x = flipFromOrigin(topLeft);
   auto y = flipFromOrigin(bottomRight);

   return QRectF(y, x);
}

QPointF Field::getOnImageCoordiante(double x, double y)
{
    return getOnImageCoordiante(QPointF(x, y));
}

QPointF Field::getOnRealCoordinate(double x, double y)
{
    return getOnRealCoordinate(QPointF(x, y));
}

QPointF Field::flipFromOrigin(double x, double y)
{
    return flipFromOrigin(QPointF(x, y));
}

double Field::getScale()
{
    return m_scale;
}




Field::~Field()
{

}

}
