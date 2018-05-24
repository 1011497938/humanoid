#include "dvision/amcl/map.hpp"
#include "dvision/amcl/amcl.hpp"
#include <QQuickPaintedItem>

namespace dmonitor {

class Map : public QQuickPaintedItem {
    Q_OBJECT
public:
    Map(QQuickItem* parent = 0);
    void paint(QPainter *painter) override;


private:
    dvision::AMCL m_amcl;
    QColor m_grassGreen = QColor(109, 178, 255);

};
}
