#pragma once

#include <etl/fsm.h>

#include "data_objects.h"
#include "messages.h"
#include "prl_defs.h"
#include "utils/atomic_bits.h"

namespace pd {

class Port; class IDriver; class PRL;

class PRL_Tx: public etl::fsm {
public:
    PRL_Tx(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_Rx: public etl::fsm {
public:
    PRL_Rx(PRL& prl);
    void log_state();
    int8_t msg_id_stored{0};
    PRL& prl;
};

class PRL_HR: public etl::fsm {
public:
    PRL_HR(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_RCH: public etl::fsm {
public:
    PRL_RCH(PRL& prl);
    void log_state();
    PRL& prl;
};

class PRL_TCH: public etl::fsm {
public:
    PRL_TCH(PRL& prl);
    void log_state();
    PRL& prl;
};

using PRL_EventListener_Base = etl::message_router<class PRL_EventListener,
    MsgSysUpdate,
    MsgToPrl_SoftResetFromPe,
    MsgToPrl_HardResetFromPe,
    MsgToPrl_PEHardResetDone,
    MsgToPrl_TcpcHardReset,
    MsgToPrl_CtlMsgFromPe,
    MsgToPrl_DataMsgFromPe,
    MsgToPrl_ExtMsgFromPe,
    MsgToPrl_GetPrlStatus>;

class PRL_EventListener : public PRL_EventListener_Base {
public:
    PRL_EventListener(PRL& prl) : PRL_EventListener_Base(ROUTER_ID::PRL), prl(prl) {}
    void on_receive(const MsgSysUpdate& msg);
    void on_receive(const MsgToPrl_SoftResetFromPe& msg);
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

    void reset_msg_counters();
    void reset_revision();

    // Mark TX chunk for sending (+ cleanup status flags from prev operations)
    void tx_enquire_chunk();
    // Fill revision / message id fields and send packet to TCPC driver
    void tcpc_enquire_msg();

    // Helper to report PE errors outside of FSM.
    void report_pending_error();

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
};

} // namespace pd
