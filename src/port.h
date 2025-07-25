#pragma once

#include <etl/message_bus.h>

#include "pe_defs.h"
#include "timers.h"
#include "utils/atomic_bits.h"

namespace pd {

namespace ROUTER_ID {
    enum {
        TASK = 1,
        TC = 2,
        PE = 3,
        PRL = 4,
        DPM = 5,
        ROUTERS_COUNT = 5
    };
} // namespace ROUTER_ID

//
// Shared data storage, message bus and utility helpers.
//
class Port {
public:
    Timers timers{};

    //
    // PE data
    //

    AtomicBits<PE_FLAG::FLAGS_COUNT> pe_flags{};
    // Micro-queue for DPM requests (set of flags)
    AtomicBits<DPM_REQUEST_FLAG::REQUEST_COUNT> dpm_requests{};

    //
    // Communication
    //

    etl::message_bus<ROUTER_ID::ROUTERS_COUNT> msgbus;

    void notify_task(etl::imessage& msg) { msgbus.receive(ROUTER_ID::TASK, msg); }
    void notify_tc(etl::imessage& msg) { msgbus.receive(ROUTER_ID::TC, msg); }
    void notify_pe(etl::imessage& msg) { msgbus.receive(ROUTER_ID::PE, msg); }
    void notify_prl(etl::imessage& msg) { msgbus.receive(ROUTER_ID::PRL, msg); }
    void notify_dpm(etl::imessage& msg) { msgbus.receive(ROUTER_ID::DPM, msg); }
};

} // namespace pd
