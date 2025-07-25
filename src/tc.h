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

class Sink;

class TC : public etl::fsm {
public:
    TC(Port& port, Sink& sink, ITCPC& tcpc);

    // Disable unexpected use
    TC() = delete;
    TC(const TC&) = delete;
    TC& operator=(const TC&) = delete;

    void dispatch(const MsgPdEvents& events);
    void log_state();
    bool is_connected();

    Port& port;
    Sink& sink;
    ITCPC& tcpc;

    // Internal vars, used from states classes
    TCPC_CC_LEVEL::Type prev_cc1{TCPC_CC_LEVEL::NONE};
    TCPC_CC_LEVEL::Type prev_cc2{TCPC_CC_LEVEL::NONE};
};

} // namespace pd