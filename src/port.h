#pragma once

#include <etl/message_bus.h>
#include <etl/cyclic_value.h>

#include "data_objects.h"
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
    // PE data /helpers
    //

    AtomicBits<PE_FLAG::FLAGS_COUNT> pe_flags{};
    // Micro-queue for DPM requests (set of flags)
    AtomicBits<DPM_REQUEST_FLAG::REQUEST_COUNT> dpm_requests{};

    PD_MSG rx_emsg{};
    PD_MSG tx_emsg{};

    PDO_LIST source_caps{};
    uint8_t hard_reset_counter{0};
    // Used to track contract type (SPR/EPR)
    uint32_t rdo_contracted{0};
    uint32_t rdo_to_request{0};

    //
    // PRL/Driver data
    //

    AtomicBits<PRL_FLAG::FLAGS_COUNT> prl_flags{};
    AtomicBits<PRL_HR_FLAG::FLAGS_COUNT> prl_hr_flags{};
    AtomicBits<PRL_TX_FLAG::FLAGS_COUNT> prl_tx_flags{};
    AtomicBits<RCH_FLAG::FLAGS_COUNT> prl_rch_flags{};
    AtomicBits<TCH_FLAG::FLAGS_COUNT> prl_tch_flags{};

    etl::cyclic_value<int8_t, 0, 7> tx_msg_id_counter{0};
    int8_t tx_retry_counter{0};
    int8_t rch_chunk_number_expected{0};
    int8_t tch_chunk_number_to_send{0};
    // Probably, single error is enouth, but let's keep separate
    // for current RCH/TCH logic.
    PRL_ERROR rch_error{};
    PRL_ERROR tch_error{};

    // shared with DRV
    PD_CHUNK rx_chunk{};
    PD_CHUNK tx_chunk{};
    etl::atomic<TCPC_TRANSMIT_STATUS::Type> tcpc_tx_status{TCPC_TRANSMIT_STATUS::UNSET};


    // In full PD stack we should keep separate revision for all SOP*.
    // But in sink we talk with charger only, and single revision for `SOP`
    // is enough.
    PD_REVISION::Type revision{PD_REVISION::REV30};

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

    bool is_prl_running();
    bool is_prl_busy();
};

} // namespace pd
