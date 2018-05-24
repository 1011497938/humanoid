#include "view.hpp"

namespace dviz {

View::View(QWidget* parent) :
    QGraphicsView(parent)
{

}

void View::mousePressEvent(QMouseEvent *event)
{
    QGraphicsView::mousePressEvent(event);
}

}
