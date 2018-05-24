#include "dmotion/dmotion.hpp"

using dmsgs::ActionCommand;
using dmsgs::BodyCommand;
namespace dmotion {
static const int MOTION_FREQ = 10;
DMotion::DMotion(ros::NodeHandle* nh)
  : DProcess(MOTION_FREQ, true)
  , m_manager(nh)
{
}

DMotion::~DMotion() = default;

void
DMotion::tick()
{
    m_manager.tick();
}

void DMotion::setCmd(ActionCommand cmd) {
    if(!m_attemptShutdown) {
        m_manager.setCmd(cmd);
    }
}

void
DMotion::prepareShutdown()
{
    ActionCommand cmd;
    cmd.bodyCmd.gait_type = BodyCommand::STANDUP;
    cmd.headCmd.pitch = cmd.headCmd.yaw = 0;

    m_manager.setCmd(cmd);
    tick();
}
}
