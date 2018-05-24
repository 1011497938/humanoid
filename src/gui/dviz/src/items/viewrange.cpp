#include "viewrange.hpp"

namespace dviz {

ViewRange::ViewRange(int id, QGraphicsItem* parent)
  : m_id(id)
  , Item(parent)
{
    m_model = Model::getInstance(id);
}

QRectF
ViewRange::boundingRect() const
{
    return QRectF(0, 0, 0, 0);
}

void
ViewRange::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    auto& viewRange = m_model->getLocFieldViewRange();
    if (viewRange.size() != 4)
        return;

    // In monitor mode, draw on loc robot,
    // else draw on sim robot

    QVector3D robotPos;
    if (m_model->getMode() == MONITOR) {
        robotPos = m_model->getLocRobotPos();
    } else {
        robotPos = m_model->getSimRobotPos();
        checkSimBallInView();
        checkCirclePointInview();
        checkGoalInview();
        checkWhitePointsInView();
        checkSimObstacleInView();
    }

    auto aa = getGlobalPosition(robotPos, viewRange[0]);
    auto bb = getGlobalPosition(robotPos, viewRange[1]);
    auto cc = getGlobalPosition(robotPos, viewRange[2]);
    auto dd = getGlobalPosition(robotPos, viewRange[3]);

    std::vector<QPointF> points{ aa, bb, cc, dd };

    drawText(aa, QString("a"));
    drawText(bb, QString("b"));
    drawText(cc, QString("c"));
    drawText(dd, QString("d"));

    QPen pen(QColor(Qt::yellow), 0);
    painter->setPen(pen);
    painter->setBrush(QBrush());
    painter->drawPolygon(points.data(), points.size());
}

// Copied from https://stackoverflow.com/a/2922778
int
pnpoly(int nvert, float* vertx, float* verty, float testx, float testy)
{
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c;
}

bool
ViewRange::inView(float x, float y)
{
    float vx[4], vy[4];
    auto& viewRange = m_model->getLocFieldViewRange();
    auto robotPos = m_model->getSimRobotPos();

    auto dx = robotPos.x() - x;
    auto dy = robotPos.y() - y;

    auto dis = sqrt(dx * dx + dy * dy);
    if (dis > MAX_SEEN_DIST)
        return false;

    auto aa = getGlobalPosition(robotPos, viewRange[0]);
    auto bb = getGlobalPosition(robotPos, viewRange[1]);
    auto cc = getGlobalPosition(robotPos, viewRange[2]);
    auto dd = getGlobalPosition(robotPos, viewRange[3]);

    vx[0] = aa.x();
    vy[0] = aa.y();
    vx[1] = bb.x();
    vy[1] = bb.y();
    vx[2] = cc.x();
    vy[2] = cc.y();
    vx[3] = dd.x();
    vy[3] = dd.y();

    return pnpoly(4, vx, vy, x, y);
}

void
ViewRange::checkSimBallInView()
{
    auto pos = Model::getSimBallPos();
    bool seen = inView(pos.x(), pos.y());
    m_model->setSeeSimball(seen);
}

void
ViewRange::checkWhitePointsInView()
{
    auto& map = m_model->getMap();
    auto occpied = map.getOccpied();

    auto& res = m_model->getSimWhitepoints();
    res.clear();

    auto robotPos = m_model->getSimRobotPos();
    geometry_msgs::Vector3 tmp;
    for (auto& o : occpied) {
        int x = o.first;
        int y = o.second;
        auto f = map.mapToField(x, y);
        if (inView(f.first, f.second)) {
            auto g = getFieldPosition(robotPos, f.first, f.second);

            tmp.x = g.x();
            tmp.y = g.y();
            res.push_back(tmp);
        }
    }
    //    qDebug() << "see" << res.size() << "white points";
}

void
ViewRange::checkCirclePointInview()
{
    m_model->setSeeSimCircle(inView(0, 0));
}

void
ViewRange::checkGoalInview()
{
    auto& res = m_model->getSimGoalPosts();
    res.clear();

    // (450, 130) (450, -130)
    // x, y
    // x, -y

    // -x, -y
    // -x, y
    auto goalX = dconstant::geometry::fieldLength / 2;
    auto goalY = dconstant::geometry::goalWidth / 2;

    auto robotPos = m_model->getSimRobotPos();
    std::vector<std::pair<int, int>> foo = { { 1, 1 }, { 1, -1 }, { -1, -1 }, { -1, 1 } };

    geometry_msgs::Vector3 tmp;
    for_each(foo.begin(), foo.end(), [&](std::pair<int, int> p) {
        auto x = p.first * goalX;
        auto y = p.second * goalY;
        if (inView(x, y)) {
            auto g = getFieldPosition(robotPos, x, y);
            tmp.x = g.x();
            tmp.y = g.y();
            res.push_back(tmp);
        }
    });
    assert(res.size() < 3);
}

void
ViewRange::checkSimObstacleInView()
{
    auto& res = m_model->getSimObstacles();
    res.clear();
    // process sim obstacle (only one)
    auto simObstaclePos = Model::getSimObstaclePos();
    bool seen = inView(simObstaclePos.x(), simObstaclePos.y());
    m_model->setSeeSimObstacle(seen);
    if (seen) {
        auto robotPos = m_model->getSimRobotPos();
        res.push_back(QPointFToVector3(getFieldPosition(robotPos, simObstaclePos)));
    }
    // TODO(corenel) treat other robots as obsatcles
}
}
