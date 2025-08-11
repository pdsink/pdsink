#pragma once

#include <etl/message_router.h>

#include "data_objects.h"
#include "messages.h"
#include "pe_defs.h"
#include "utils/atomic_bits.h"
#include "utils/tick_fsm.h"

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


class PE : public etl_ext::tick_fsm<PE> {
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

    void check_request_progress_enter();
    auto check_request_progress_run() -> PE_REQUEST_PROGRESS;
    void check_request_progress_exit();

    enum class LOCAL_STATE {
        DISABLED, INIT, WORKING
    } local_state{LOCAL_STATE::DISABLED};

    Port& port;
    IDPM& dpm;
    PRL& prl;
    ITCPC& tcpc;

    PE_EventListener pe_event_listener;
};

} // namespace pd
