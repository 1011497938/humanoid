#pragma once
#include <QQuickPaintedItem>
#include "dconfig/dconstant.hpp"
#include <opencv2/opencv.hpp>
#include "dmonitor/utils.hpp"

// TODO(MWX): field config


using namespace dconstant::geometry;
namespace dmonitor {
class Field : public QQuickPaintedItem {
    Q_OBJECT
public:
    Field(QQuickItem* parent = 0);
    ~Field();
    void paint(QPainter *painter) override;
    void drawField(QPainter* painter);
    Q_INVOKABLE void init();

    // convert point's coordinate from real(center of the field) to image (up left corner) for drawing
    QPointF getOnImageCoordiante(QPointF real);
    QPointF getOnImageCoordiante(cv::Point2f real);
    QPointF getOnRealCoordinate(QPointF img);
    QPointF getOnRealCoordinate(cv::Point2f img);
    QPointF flipFromOrigin(QPointF p);
    QRectF flipFromOrigin(QRectF rect);

    QPointF getOnImageCoordiante(double x, double y);
    QPointF getOnRealCoordinate(double x, double y);
    QPointF flipFromOrigin(double x, double y);
    double getScale();

    void drawCross(QPainter *painter, QPointF real);

    void dragEnterEvent(QDragEnterEvent *);
    void mouseMoveEvent(QMouseEvent *event);
private:
    // Field geometry in cm, refer to RoboCup Humanoid Rules
    int m_fieldLength = fieldLength;
    int m_fieldWidth = fieldWidth;
    int m_goalDepth = goalDepth;
    int m_goalWidth = goalWidth;
    int m_goalHeight = goalHeight;
    int m_goalAreaLength = goalAreaLength;
    int m_goalAreaWidth = goalAreaWidth;
    int m_penaltyMarkDistance = penaltyMarkDistance;
    int m_centerCircleDiameter = centerCircleDiameter;
    int m_borderStripWidth = borderStripWidth;
    int m_lineWidth = lineWidth;

    int m_imageWidth = m_fieldLength + m_borderStripWidth * 2;
    int m_imageHeight = m_fieldWidth + m_borderStripWidth * 2;
    double m_imageRatio = (double)m_imageWidth / m_imageHeight;
    double m_scale = 1;

    // color
    QColor m_grassGreen = QColor(109, 178, 255);
};
}
