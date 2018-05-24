#pragma once
#include <QtGui>
#include <QGraphicsView>

namespace dviz {

class View : public QGraphicsView {
    Q_OBJECT
public:
    View(QWidget* parent = 0);


protected:
    void mousePressEvent(QMouseEvent* event) override;

};

}
