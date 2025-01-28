//
// Type-C port manager.
//
// For our case - used only for monitoring CC0/CC1/VBUS states, to
// activate/deactivate PD protocol stack.
//
#pragma once

#include "data_objects.h"
#include "idriver.h"

#include <etl/fsm.h>

namespace pd {

class Sink;

class TC : public etl::fsm {
public:
    TC(Sink& sink, ITCPC& tcpc);
    void dispatch(const MsgPdEvents& events);
    void log_state();
    bool is_connected();

    // Disable unexpected use
    TC() = delete;
    TC(const TC&) = delete;
    TC& operator=(const TC&) = delete;

    Sink& sink;
    ITCPC& tcpc;

    // Internal vars, used from states classes
    TCPC_CC_LEVEL::Type prev_cc1{TCPC_CC_LEVEL::NONE};
    TCPC_CC_LEVEL::Type prev_cc2{TCPC_CC_LEVEL::NONE};
};

} // namespace pd