#pragma once
#include "dmonitor/baseObject.hpp"
#include "dtransmit/dtransmit.hpp"
#include <QImage>
#include <QPixmap>

namespace dmonitor {

class GuiImage : public BaseObject {
    Q_OBJECT
    Q_PROPERTY(int currentId READ currentId WRITE setCurrentId NOTIFY currentIdChanged)

public:
    GuiImage(QQuickItem* parent = 0);
    void init() override;
    void drawMyself(QPainter *painter) override;

    int currentId() const;

public slots:
    void setCurrentId(int currentId);

signals:
    void currentIdChanged(int currentId);

private:
    dtransmit::DTransmit* m_transmitter;
    QPixmap m_image;
    int m_currentId;
};

} // namespace dmonitor
