#include "render.hpp"
#include <QtCore>
#include <QDebug>
#include <QTimer>
#include <QGraphicsItem>
#include <dconfig/dconstant.hpp>


namespace dviz {

Render::Render(QObject *parent)
    : QGraphicsScene(parent)
{
    this->setBackgroundBrush(QColor(109, 178, 255));
    int w = dconstant::geometry::wholeWidth;
    int h = dconstant::geometry::wholeHeight;
    this->setSceneRect(-w / 2, -h / 2, w, h);
}

Render::~Render() {
}

}
