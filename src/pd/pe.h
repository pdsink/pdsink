#pragma once

#include <etl/message_router.h>

#include "data_objects.h"
#include "messages.h"
#include "pe_defs.h"
#include "utils/atomic_bits.h"
#include "utils/afsm.h"

namespace pd {

enum class PE_REQUEST_PROGRESS {
    PENDING,
    FINISHED,
    DISCARDED,
    FAILED
};

class Port; class IDPM; class ITCPC; class PE; class PRL;

using PE_EventListener_Base = etl::message_router<class PE_EventListener,
    MsgSysUpdate,
    MsgToPe_PrlMessageReceived,
    MsgToPe_PrlMessageSent,
    MsgToPe_PrlReportError,
    MsgToPe_PrlReportDiscard,
    MsgToPe_PrlSoftResetFromPartner,
    MsgToPe_PrlHardResetFromPartner,
    MsgToPe_PrlHardResetSent>;

class PE_EventListener : public PE_EventListener_Base {
public:
    PE_EventListener(PE& pe) : pe(pe) {}
    void on_receive(const MsgSysUpdate& msg);
    void on_receive(const MsgToPe_PrlMessageReceived& msg);
    void on_receive(const MsgToPe_PrlMessageSent& msg);
    void on_receive(const MsgToPe_PrlReportError& msg);
    void on_receive(const MsgToPe_PrlReportDiscard& msg);
    void on_receive(const MsgToPe_PrlSoftResetFromPartner& msg);
    void on_receive(const MsgToPe_PrlHardResetFromPartner& msg);
    void on_receive(const MsgToPe_PrlHardResetSent& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    PE& pe;
};


class PE : public afsm::fsm<PE> {
public:
    PE(Port& port, IDPM& dpm, PRL& prl, ITCPC& tcpc);

    // Disable unexpected use
    PE() = delete;
    PE(const PE&) = delete;
    PE& operator=(const PE&) = delete;

    void log_state() const;
    void log_source_caps() const;
    void setup();
    void init();
    void request_wakeup() { has_deferred_wakeup_request = true; };

    // Helpers
    void send_ctrl_msg(PD_CTRL_MSGT::Type msgt);
    void send_data_msg(PD_DATA_MSGT::Type msgt);
    void send_ext_msg(PD_EXT_MSGT::Type msgt);

    bool is_in_epr_mode() const;
    bool is_in_spr_contract() const;
    bool is_in_pps_contract() const;
    bool is_epr_mode_available() const;
    bool validate_source_caps(const etl::ivector<uint32_t>& src_caps);

    enum class LOCAL_STATE {
        DISABLED, INIT, WORKING
    } local_state{LOCAL_STATE::DISABLED};

    Port& port;
    IDPM& dpm;
    PRL& prl;
    ITCPC& tcpc;

    DPM_REQUEST_FLAG active_dpm_request{DPM_REQUEST_FLAG::NONE};
    PE_REQUEST_PROGRESS request_progress{PE_REQUEST_PROGRESS::PENDING};

    PE_EventListener pe_event_listener;
    etl::atomic<bool> has_deferred_wakeup_request{false};
};

} // namespace pd
