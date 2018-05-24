#include "dprocess/dprocess.hpp"
#include "dmotion/dmotion.hpp"

using dprocess::DProcess;
namespace dmotion {

class BehaviorRecv : public DProcess<BehaviorRecv> {
public:
    explicit BehaviorRecv(ros::NodeHandle* n, DMotion* d);
    void tick() override;

private:
    void callback(const dmsgs::ActionCommand::ConstPtr& msg);
    ros::NodeHandle* m_nh;
    DMotion* m_dmotion;
    ros::Subscriber m_sub;
};

}