#include <QPainter>
#include "dmonitor/baseObject.hpp"

namespace dmonitor {
BaseObject::BaseObject(QQuickItem *parent) : QQuickPaintedItem(parent) {
    connect(this, &BaseObject::xChanged, this, &BaseObject::onxyChanged);
    connect(this, &BaseObject::yChanged, this, &BaseObject::onxyChanged);
}

BaseObject::~BaseObject() {

}

void BaseObject::init()
{

}

Field* BaseObject::field() const {
    return m_field;
}

bool BaseObject::isMonitor() const {
    return m_isMonitor;
}

void BaseObject::paint(QPainter *painter)
{
   if(!m_enabled)
       return;

   if(!m_field) {
       qDebug() << "Field not set!" << endl;
       return;
   }

   painter->setRenderHint(QPainter::Antialiasing);

   if(m_isMonitor) {
       monitorModeUpdate();
   } else {
       simModeUpdate();
   }
   drawMyself(painter);
}

bool BaseObject::enabled() const
{
    return m_enabled;
}

void BaseObject::setField(Field *field) {
    m_field = field;
}

void BaseObject::setIsMonitor(bool isMonitor) {
    m_isMonitor = isMonitor;
}

void BaseObject::onxyChanged()
{

}

void BaseObject::monitorModeUpdate()
{
}

void BaseObject::simModeUpdate()
{
}

void BaseObject::setEnable(bool enabled)
{
    m_enabled = enabled;
}

int BaseObject::robotId() const
{
    return m_robotId;
}

void BaseObject::setRobotId(int id)
{
    if (m_robotId == id)
        return;

    m_robotId = id;
}

void BaseObject::setPos(QPointF p) {
    m_realPos = p;
}
} // namespace dmonitor
