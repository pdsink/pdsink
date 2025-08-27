#pragma once

#include "data_objects.h"
#include "messages.h"
#include "prl_defs.h"
#include "utils/atomic_bits.h"
#include "utils/afsm.h"

namespace pd {

class Port; class IDriver; class PRL;

class PRL_Tx: public afsm::fsm<PRL_Tx> {
public:
    PRL_Tx(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_Rx: public afsm::fsm<PRL_Rx> {
public:
    PRL_Rx(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_HR: public afsm::fsm<PRL_HR> {
public:
    PRL_HR(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_RCH: public afsm::fsm<PRL_RCH> {
public:
    PRL_RCH(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_TCH: public afsm::fsm<PRL_TCH> {
public:
    PRL_TCH(PRL& prl);
    void log_state();
    PRL& prl;
};

using PRL_EventListener_Base = etl::message_router<class PRL_EventListener,
    MsgSysUpdate,
    MsgToPrl_EnquireRestart,
    MsgToPrl_HardResetFromPe,
    MsgToPrl_PEHardResetDone,
    MsgToPrl_TcpcHardReset,
    MsgToPrl_CtlMsgFromPe,
    MsgToPrl_DataMsgFromPe,
    MsgToPrl_ExtMsgFromPe,
    MsgToPrl_GetPrlStatus>;

class PRL_EventListener : public PRL_EventListener_Base {
public:
    PRL_EventListener(PRL& prl) : prl(prl) {}
    void on_receive(const MsgSysUpdate& msg);
    void on_receive(const MsgToPrl_EnquireRestart& msg);
    void on_receive(const MsgToPrl_HardResetFromPe& msg);
    void on_receive(const MsgToPrl_PEHardResetDone& msg);
    void on_receive(const MsgToPrl_TcpcHardReset& msg);
    void on_receive(const MsgToPrl_CtlMsgFromPe& msg);
    void on_receive(const MsgToPrl_DataMsgFromPe& msg);
    void on_receive(const MsgToPrl_ExtMsgFromPe& msg);
    void on_receive(const MsgToPrl_GetPrlStatus& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    PRL& prl;
};

class PRL {
public:
    PRL(Port& port, IDriver& tcpc);

    // Disable unexpected use
    PRL() = delete;
    PRL(const PRL&) = delete;
    PRL& operator=(const PRL&) = delete;

    void setup();
    void init(bool from_hr_fsm = false);
    void request_wakeup() { has_deferred_wakeup_request = true; };
    // notify + deferred wakeup
    void report_pe(const etl::imessage& msg);

    void reset_msg_counters();
    void reset_revision();

    // Mark TX chunk for sending (+ cleanup status flags from prev operations)
    void prl_tx_enquire_chunk();

    enum class LOCAL_STATE {
        DISABLED, INIT, WORKING
    } local_state{LOCAL_STATE::DISABLED};

    Port& port;
    IDriver& tcpc;

    PRL_Tx prl_tx;
    PRL_Rx prl_rx;
    PRL_HR prl_hr;
    PRL_RCH prl_rch;
    PRL_TCH prl_tch;

    PRL_EventListener prl_event_listener;
    etl::atomic<bool> has_deferred_wakeup_request{false};
};

} // namespace pd
