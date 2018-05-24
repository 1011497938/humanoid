#include "dmotion/BehaviorRecv.hpp"

namespace dmotion {
BehaviorRecv::BehaviorRecv(ros::NodeHandle* n, DMotion *d)
    : DProcess(100, false), m_nh(n), m_dmotion(d) {
    int robotId;
    if(!m_nh->getParam("RobotId", robotId))
        throw std::runtime_error("Can't get robot id");

    m_sub = m_nh->subscribe("/dbehavior_" + std::to_string(robotId) + "/ActionCommand", 1, &BehaviorRecv::callback, this);
}

void BehaviorRecv::callback(const dmsgs::ActionCommand::ConstPtr& msg) {
    m_dmotion->setCmd(*msg);
}

void BehaviorRecv::tick() {
    // pass
}
} // namespace dmotion