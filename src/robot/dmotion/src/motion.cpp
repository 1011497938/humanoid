#include "motion.hpp"
#include "MotionConfig.hpp"
#include "dmotion/GaitStateLib/GaitStateManager.cpp"
#include "misc/configclient/configclient.hpp"
#include "misc/endpoints.hpp"
#include "misc/node/node.hpp"
#include "misc/utils/logger/logger.hpp"
#include <functional>

using namespace std;
using namespace dancer2050;

static const string CFG_SERVER_FILE = "motion.cfg";
static const string BEHAVIOUR_TOPIC = "ActionCommand";

ActionCommand::All behave_req; // global request
Node mynode(Endpoint::MOTION);

Motion::Motion()
  : Process<Motion>(true)
{
}

Motion::~Motion()
{
}

void
Motion::tick()
{
    Logger<Console>::init(DEBUG);
    auto cfg_client = MotionConfigClient::getinstance(CFG_SERVER_FILE, Endpoint::CFG_PUB, Endpoint::CFG_REPLY);
    GaitStateManager manager;

    mynode.nonblock_sub(Endpoint::BEHAVIOUR, "ActionCommand", [&](string data) { behave_req = ActionCommand::All::restore(data); });

    while (true) {
        cfg_client->update();

        if (cfg_client->reload_gaitdata) {
            manager.reload_gaitdata();
            cfg_client->reload_gaitdata = false;
        }

        mynode.poll();
        manager.checkNewCommand(behave_req);
        manager.tick();
    }
}
