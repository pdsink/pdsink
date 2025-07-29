#pragma once

#include <etl/fsm.h>

#include "data_objects.h"
#include "messages.h"
#include "pe_defs.h"
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
    PE_EventListener(PE& pe) : PE_EventListener_Base(ROUTER_ID::PE), pe(pe) {}
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


class PE : public etl::fsm {
public:
    PE(Port& port, IDPM& dpm, PRL& prl, ITCPC& tcpc);

    // Disable unexpected use
    PE() = delete;
    PE(const PE&) = delete;
    PE& operator=(const PE&) = delete;

    void log_state();
    void setup();
    void init();

    // Helpers
    void send_ctrl_msg(PD_CTRL_MSGT::Type msgt);
    void send_data_msg(PD_DATA_MSGT::Type msgt);
    void send_ext_msg(PD_EXT_MSGT::Type msgt);

    bool is_in_epr_mode() const;
    bool is_in_spr_contract() const;
    bool is_in_pps_contract() const;
    bool is_epr_mode_available() const;
    bool is_rev_2_0() const;

    void check_request_progress_enter();
    auto check_request_progress_run() -> PE_REQUEST_PROGRESS::Type;
    void check_request_progress_exit();

    enum { LS_DISABLED, LS_INIT, LS_WORKING } local_state = LS_DISABLED;

    Port& port;
    IDPM& dpm;
    PRL& prl;
    ITCPC& tcpc;

    PE_EventListener pe_event_listener;
};

} // namespace pd
