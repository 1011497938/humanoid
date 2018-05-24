#pragma once
#include "item.hpp"
namespace dviz {

static const float MAX_SEEN_DIST = 500;

class ViewRange : public Item
{
  public:
    ViewRange(int id, QGraphicsItem* parent = 0);

    QRectF boundingRect() const;
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

    bool inView(float x, float y);
    void checkSimBallInView();
    void checkWhitePointsInView();
    void checkCirclePointInview();
    void checkGoalInview();
    void checkSimObstacleInView();

  private:
    int m_id;
};
}
