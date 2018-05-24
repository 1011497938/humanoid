#include "dmonitor/guiImage.hpp"
#include "dvision/frame.hpp"
#include <QPainter>

using namespace dtransmit;
namespace dmonitor {

GuiImage::GuiImage(QQuickItem *parent) : BaseObject(parent) {

}

void GuiImage::init() {
    dvision::Frame::initEncoder();
    m_transmitter = new DTransmit("192.168.255.255");


    for(int i = 1; i <= 6; ++i) {
        m_transmitter->addRawRecv(dconstant::network::robotGuiBase + i , [&, i](void* buffer, std::size_t size) {
            dvision::Frame f;
            if(m_currentId != i)
                return;
            try {
                f.decode(buffer);
                auto img = f.getRGB();
                m_image = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888));
            } catch (std::exception& e) {
                std::cerr << e.what() << std::endl;
            }
        });
    }

    m_transmitter->startService();
}

void GuiImage::drawMyself(QPainter *painter)
{
//    qDebug() << "gui draw";
//    painter->setPen(QPen(Qt::black, 10));
//    painter->drawPoint(0, 0);
    painter->drawPixmap(0, 0, width(), height(), m_image);
}

int GuiImage::currentId() const
{
    return m_currentId;
}

void GuiImage::setCurrentId(int currentId)
{
    if (m_currentId == currentId)
        return;

    m_currentId = currentId;
    emit currentIdChanged(currentId);
}


} // namespace dmonitor
