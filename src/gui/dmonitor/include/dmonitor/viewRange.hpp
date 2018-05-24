#pragma once
#include "dmonitor/baseObject.hpp"
#include "dmsgs/VisionInfo.h"
#include <QtCore>

namespace dmonitor {

class ViewRange : public BaseObject {
    Q_OBJECT
public:
    ViewRange(QQuickItem* parent = 0);
    void drawMyself(QPainter* painter) override;
    void setVisionInfo(dmsgs::VisionInfo info);

    bool inView(float x, float y);
private:
    dmsgs::VisionInfo m_visionInfo;

};

} //namespace dmonitor
