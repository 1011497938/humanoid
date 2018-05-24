#pragma once
#include <opencv2/opencv.hpp>
#include <QtCore>
using namespace cv;
namespace dmonitor {

template <typename T>
inline QPointF getQPoint(T p) {
    return QPointF(p.x, p.y);
}

} // namespace dmonitor
