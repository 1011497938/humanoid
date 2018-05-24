#pragma once
#include <QObject>
#include <QGraphicsScene>

namespace dviz {

class ControlWidget;
class ItemManager : public QObject {
    Q_OBJECT
public:
    ItemManager(QObject* parent, QGraphicsScene* scene, ControlWidget* control);
    ~ItemManager();
    void Init();

private:
    int cycle_ = 0;
    QGraphicsScene* m_scene;
    ControlWidget* m_control;
};

}
