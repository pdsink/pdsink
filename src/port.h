#pragma once

#include <etl/message_bus.h>

#include "pe_defs.h"
#include "messages.h"
#include "timers.h"
#include "utils/atomic_bits.h"

namespace pd {

//
// Shared data storage, message bus and utility helpers.
//

class Port {
public:
    Timers timers{};

    bool is_attached{false};

    //
    // PE data
    //

    AtomicBits<PE_FLAG::FLAGS_COUNT> pe_flags{};
    // Micro-queue for DPM requests (set of flags)
    AtomicBits<DPM_REQUEST_FLAG::REQUEST_COUNT> dpm_requests{};

    PD_MSG rx_emsg{};
    PD_MSG tx_emsg{};

    PDO_LIST source_caps{};

    //
    // PRL data
    //

    PD_CHUNK rx_chunk{};
    PD_CHUNK tx_chunk{};

    //
    // Communication
    //

    etl::message_bus<ROUTER_ID::ROUTERS_COUNT> msgbus{};

    void notify_task(const etl::imessage& msg);
    void notify_tc(const etl::imessage& msg);
    void notify_pe(const etl::imessage& msg);
    void notify_prl(const etl::imessage& msg);
    void notify_dpm(const etl::imessage& msg);
    void wakeup();

    //
    // Helpers
    //

    bool is_ams_active();

    void wait_dpm_transit_to_default(bool enable);
};

} // namespace pd
