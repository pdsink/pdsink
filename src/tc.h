//
// Type-C port manager.
//
// For our case - used only for monitoring CC0/CC1/VBUS states, to
// activate/deactivate PD protocol stack.
//
#pragma once

#include <etl/fsm.h>

#include "data_objects.h"
#include "idriver.h"
#include "port.h"

namespace pd {

class Task;

using TC_EventListener_Base = etl::message_router<class TC_EventListener, MsgSysUpdate>;

class TC_EventListener : public TC_EventListener_Base {
public:
    TC_EventListener(TC& tc) : TC_EventListener_Base(ROUTER_ID::TC), tc(tc) {}
    void on_receive(const MsgSysUpdate& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    TC& tc;
};

class TC : public etl::fsm {
public:
    TC(Port& port, Task& task, ITCPC& tcpc);

    // Disable unexpected use
    TC() = delete;
    TC(const TC&) = delete;
    TC& operator=(const TC&) = delete;

    void log_state();

    Port& port;
    Task& task;
    ITCPC& tcpc;

    // Internal vars, used from states classes
    TCPC_CC_LEVEL::Type prev_cc1{TCPC_CC_LEVEL::NONE};
    TCPC_CC_LEVEL::Type prev_cc2{TCPC_CC_LEVEL::NONE};

    TC_EventListener tc_event_listener;
};

} // namespace pd