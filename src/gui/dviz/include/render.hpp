#pragma once
#include <QGraphicsScene>


namespace dviz {

/* @brief Render
 */
class Render : public QGraphicsScene {
    Q_OBJECT
public:
    Render(QObject* parent = 0);
    ~Render();
};


}
