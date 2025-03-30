#pragma once

#include "common.h"
#include "idriver.h"
#include "prl.h"
#include "utils/atomic_bits.h"

#include <etl/fsm.h>
#include <etl/vector.h>

namespace pd {

// Boolean flags to organize  simple DPM requests queue
namespace DPM_REQUEST {
    enum Type {
        UNUSED, // Skip 0 to simplify current request checks
        NEW_POWER_LEVEL,
        EPR_MODE_ENTRY,

        REQUEST_COUNT
    };
}; // namespace DPM_REQUEST

namespace PE_REQUEST_PROGRESS {
    enum Type {
        PENDING,
        FINISHED,
        DISCARDED,
        FAILED
    };
}; // namespace PE_REQUEST_PROGRESS

class Sink;
class PRL;

namespace PE_FLAG {
    enum Type {
        // Flags of message transfer (set by PRL)
        TX_COMPLETE,   // Message sent
        MSG_DISCARDED, // Outgoing message discarded by new incoming one
        MSG_RECEIVED,  // Got reply OR new message (which discarded outgoing transfer)

        // By default PRL error usually causes soft reset. This flag can be set
        // at state entry when custom handling needed. Then error will rise
        // PROTOCOL_ERROR flag instead.
        FORWARD_PRL_ERROR,
        PROTOCOL_ERROR,

        HAS_EXPLICIT_CONTRACT,
        IN_EPR_MODE,
        SNK_WAIT_CAP_TIMEOUT,
        AMS_ACTIVE,
        AMS_FIRST_MSG_SENT,

        // Minor local flags to control states behaviour
        PRL_HARD_RESET_PENDING,
        IS_FROM_EVALUATE_CAPABILITY,
        DO_SOFT_RESET_ON_UNSUPPORTED,
        CAN_SEND_SOFT_RESET,
        TRANSMIT_REQUEST_SUCCEEDED,

        FLAGS_COUNT
    };
};

class PE : public etl::fsm {
public:
    PE(Sink& sink, PRL& prl, ITCPC& tcpc);
    void log_state();
    void init();
    void dispatch(const MsgPdEvents& events, const bool pd_enabled);

    PD_MSG& get_rx_msg();
    PD_MSG& get_tx_msg();

    // Notification handlers (from PRL)
    void on_message_received();
    void on_message_sent();
    void on_prl_report_error(PRL_ERROR err);
    void on_prl_report_discard();

    void on_prl_soft_reset_from_partner();
    void on_prl_hard_reset_from_partner();
    void on_prl_hard_reset_sent();

    bool is_ams_requested() { return flags.test(PE_FLAG::AMS_ACTIVE); }

    // Helpers
    void send_ctrl_msg(PD_CTRL_MSGT::Type msgt);
    void send_data_msg(PD_DATA_MSGT::Type msgt);
    void send_ext_msg(PD_EXT_MSGT::Type msgt);

    bool is_in_epr_mode() const { return flags.test(PE_FLAG::IN_EPR_MODE); }
    bool is_in_spr_contract() const;
    bool is_in_pps_contract() const;
    bool is_epr_mode_available() const;
    bool is_rev_2_0() const;

    void check_request_progress_enter();
    auto check_request_progress_run() -> PE_REQUEST_PROGRESS::Type;
    void check_request_progress_exit();

    // Disable unexpected use
    PE() = delete;
    PE(const PE&) = delete;
    PE& operator=(const PE&) = delete;

    enum { LS_DISABLED, LS_INIT, LS_WORKING } local_state = LS_DISABLED;

    AtomicBits<PE_FLAG::FLAGS_COUNT> flags{};
    uint8_t hard_reset_counter{0};
    PDO_LIST source_caps{};

    // Used to track contract type (SPR/EPR)
    uint32_t rdo_contracted{0};
    uint32_t rdo_to_request{0};

    //
    // Micro-queue for DPM requests (set of flags)
    //
    AtomicBits<DPM_REQUEST::REQUEST_COUNT> dpm_requests{};
    // dpm_request ID (bit number from dpm_requests), running now
    uint32_t dpm_current_request{0};

    Sink& sink;
    PRL& prl;
    ITCPC& tcpc;
};

} // namespace pd
