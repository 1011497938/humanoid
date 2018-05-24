#include "dmonitor/viewRange.hpp"
#include <QPainter>

namespace dmonitor {

ViewRange::ViewRange(QQuickItem *parent) : BaseObject(parent)
{
}

void ViewRange::drawMyself(QPainter *painter)
{
    if(m_visionInfo.viewRange.size() != 4){
        return;
    }

    setX(0);
    setY(0);
    setWidth(m_field->width());
    setHeight(m_field->height());

    QPointF a(m_visionInfo.viewRange[0].x, m_visionInfo.viewRange[0].y);
    QPointF b(m_visionInfo.viewRange[1].x, m_visionInfo.viewRange[1].y);
    QPointF c(m_visionInfo.viewRange[2].x, m_visionInfo.viewRange[2].y);
    QPointF d(m_visionInfo.viewRange[3].x, m_visionInfo.viewRange[3].y);

//    qDebug() << a << b << c << d;

    auto aa = m_field->getOnImageCoordiante(a);
    auto bb = m_field->getOnImageCoordiante(b);
    auto cc = m_field->getOnImageCoordiante(c);
    auto dd = m_field->getOnImageCoordiante(d);

    std::vector<QPointF> points {
        aa, bb, cc, dd
    };

    painter->drawText(aa, QString("a"));
    painter->drawText(bb, QString("b"));
    painter->drawText(cc, QString("c"));
    painter->drawText(dd, QString("d"));

    QPen pen(QColor(Qt::yellow), 0);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    painter->drawPolygon(points.data(), points.size());
}

// Copied from https://stackoverflow.com/a/2922778
int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

bool ViewRange::inView(float x, float y) {
    float vx[4], vy[4];

    vx[0] = m_visionInfo.viewRange[0].x;
    vy[0] = m_visionInfo.viewRange[0].y;

    vx[1] = m_visionInfo.viewRange[1].x;
    vy[1] = m_visionInfo.viewRange[1].y;

    vx[2] = m_visionInfo.viewRange[2].x;
    vy[2] = m_visionInfo.viewRange[2].y;

    vx[3] = m_visionInfo.viewRange[3].x;
    vy[3] = m_visionInfo.viewRange[3].y;

    return pnpoly(4, vx, vy, x, y);

}

void ViewRange::setVisionInfo(dmsgs::VisionInfo info)
{
    m_visionInfo = info;
}

}
