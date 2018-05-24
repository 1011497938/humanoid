#include "clabel.hpp"
#include <QMouseEvent>
#include <QtCore>
#include <QPainter>
#include <QPen>


CLabel::CLabel(QWidget *parent) : QLabel(parent)
{
    connect(this, &CLabel::clicked,
            this, &CLabel::updateView);
}

void CLabel::setModel(MyModel *model)
{
    m_listmodel = model;
}

void CLabel::mouseReleaseEvent(QMouseEvent *ev)
{
    mouseX = ev->x();
    mouseY = ev->y();
    emit clicked(ev);
}

void CLabel::mouseMoveEvent(QMouseEvent *ev)
{

    mouseX = ev->x();
    mouseY = ev->y();

    emit clicked(ev);
}

void CLabel::updateView()
{
    // get current img
    QPixmap tmp = m_listmodel->getImage();

    // paint
    QPainter painter(&tmp);
    QPen pen(Qt::red);
    pen.setWidth(3);
    QPoint p1;
    p1.setX(mouseX);
    p1.setY(mouseY);
    painter.setPen(pen);
    painter.drawPoint(p1);
    setPixmap(tmp);

//    qDebug() << "Original: (" << mouseX << ", " << mouseY << ")";
}

