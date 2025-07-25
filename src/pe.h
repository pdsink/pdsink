#pragma once

#include <etl/fsm.h>

#include "data_objects.h"
#include "idriver.h"
#include "messages.h"
#include "pe_defs.h"
#include "port.h"
#include "prl.h"
#include "utils/atomic_bits.h"

namespace pd {

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

class PE : public etl::fsm {
public:
    PE(Port& port, Sink& sink, PRL& prl, ITCPC& tcpc);

    // Disable unexpected use
    PE() = delete;
    PE(const PE&) = delete;
    PE& operator=(const PE&) = delete;

    void log_state();
    void init();
    void dispatch(const MsgSysUpdate& events, const bool pd_enabled);

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

    bool is_ams_active() { return flags.test(PE_FLAG::AMS_ACTIVE); }
    void wait_dpm_transit_to_default(bool enable);

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
    AtomicBits<DPM_REQUEST_FLAG::REQUEST_COUNT> dpm_requests{};

    Port& port;
    Sink& sink;
    PRL& prl;
    ITCPC& tcpc;
};

} // namespace pd
