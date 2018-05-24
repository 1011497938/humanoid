#include "dmonitor/map.hpp"
#include <QPainter>
#include <QDebug>

namespace dmonitor {

Map::Map(QQuickItem *parent) : QQuickPaintedItem(parent) {
    m_amcl.Init();
}

void Map::paint(QPainter *painter) {
    painter->setRenderHint(QPainter::Antialiasing);
    painter->fillRect(0, 0, width(), height(), QColor(Qt::white));

    QColor black = QColor(Qt::red);
    QColor white = QColor(Qt::white);

    auto& map = m_amcl.map();
    int maxOccDist = map.maxOccDist();

    for(int x = 0; x < map.width(); ++x) {
        for(int y = 0; y < map.height(); ++y) {
            auto& c = map.getCell(x, y);
            int dist = c.occ_dist;

            //qDebug() << dist << (1.0 * dist / maxOccDist) * 100;

            painter->fillRect(x * 5 + 1, y * 5 + 1, 4, 4, black.lighter(100 + 1.0f * dist / maxOccDist * 100));
//            painter->fillRect(x * 5 + 1, y * 5 + 1, 4, 4, black.lighter(200));

//            if(dist == 0) {
//                if(x > 100)
//                    painter->fillRect(x * 5 + 1, y * 5 + 1, 4, 4, black.lighter(100 + 1.0f * dist / maxOccDist * 100));
//                else
//                    painter->fillRect(x * 5 + 1, y * 5 + 1, 4, 4, black);
//            }
        }
    }
}


// max --> 100
// 0 --> 0

} // namespace dmonitor
