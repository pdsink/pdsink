#pragma once

#include <etl/cyclic_value.h>
#include <etl/fsm.h>

#include "data_objects.h"
#include "idriver.h"
#include "messages.h"
#include "port.h"
#include "prl_defs.h"
#include "utils/atomic_bits.h"

namespace pd {

class Sink;
class PRL;

namespace PRL_FLAG {
    enum Type {
        // This is implemented according to spec for consistency, but not
        // actually used in public API
        ABORT,
        FLAGS_COUNT
    };
}; // namespace PRL_FLAG

namespace RCH_FLAG {
    enum Type {
        RX_ENQUEUED, // From RX
        RCH_ERROR_PENDING,
        FLAGS_COUNT
    };
}; // namespace RCH_FLAG

namespace TCH_FLAG {
    enum Type {
        MSG_ENQUEUED, // From PE
        NEXT_CHUNK_REQUEST,
        TCH_ERROR_PENDING,
        FLAGS_COUNT
    };
}; // namespace TCH_FLAG

namespace PRL_TX_FLAG {
    enum Type {
        TX_CHUNK_ENQUEUED, // From TCH/RCH
        TX_COMPLETED,
        TX_DISCARDED,
        TX_ERROR,
        START_OF_AMS_DETECTED,
        FLAGS_COUNT
    };
}; // namespace PRL_TX_FLAG

namespace PRL_HR_FLAG {
    enum Type {
        HARD_RESET_FROM_PARTNER,
        HARD_RESET_FROM_PE,
        PE_HARD_RESET_COMPLETE,
        FLAGS_COUNT
    };
}; // namespace PRL_HR_FLAG

class PRL_Tx: public etl::fsm {
public:
    PRL_Tx(PRL& prl);
    void log_state();
    AtomicBits<PRL_TX_FLAG::FLAGS_COUNT> flags{};
    etl::cyclic_value<int8_t, 0, 7> msg_id_counter{0};
    int8_t retry_counter{0};
    TCPC_TRANSMIT_STATUS::Type tcpc_tx_status{TCPC_TRANSMIT_STATUS::UNSET};
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
    AtomicBits<PRL_HR_FLAG::FLAGS_COUNT> flags{};
    PRL& prl;
};

class PRL_RCH: public etl::fsm {
public:
    PRL_RCH(PRL& prl);
    void log_state();
    AtomicBits<RCH_FLAG::FLAGS_COUNT> flags{};
    int8_t chunk_number_expected{0};
    PRL_ERROR error{};
    PRL& prl;
};

class PRL_TCH: public etl::fsm {
public:
PRL_TCH(PRL& prl);
    void log_state();
    AtomicBits<TCH_FLAG::FLAGS_COUNT> flags{};
    int8_t chunk_number_to_send{0};
    PRL_ERROR error{};
    PRL& prl;
};

using PRL_EventListener_Base = etl::message_router<class PRL_EventListener,
    MsgSysUpdate,
    MsgToPrl_SoftResetFromPe,
    MsgToPrl_HardResetFromPe,
    MsgToPrl_PEHardResetDone,
    MsgToPrl_TcpcHardReset,
    MsgToPrl_TcpcTransmitStatus>;

class PRL_EventListener : public PRL_EventListener_Base {
public:
    PRL_EventListener(PRL& prl) : PRL_EventListener_Base(ROUTER_ID::PRL), prl(prl) {}
    void on_receive(const MsgSysUpdate& msg);
    void on_receive(const MsgToPrl_SoftResetFromPe& msg);
    void on_receive(const MsgToPrl_HardResetFromPe& msg);
    void on_receive(const MsgToPrl_PEHardResetDone& msg);
    void on_receive(const MsgToPrl_TcpcHardReset& msg);
    void on_receive(const MsgToPrl_TcpcTransmitStatus& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    PRL& prl;
};

class PRL {
public:
    PRL(Port& port, Sink& sink, IDriver& tcpc);
    void init(bool from_hr_fsm = false);

    // False if unchunked ext messages supported by both partners. In our case
    // always True.
    //
    // - Not critical to support both, and chunking is more compatible.
    // - I theory, unchunked ext are more simple, without TCH/RCH, BUT (!) FUSB302
    //   has limited fifo capacity, and will need more complicated driver logic
    //
    // This decision can be revisited later, if time allows.
    bool chunking() { return true; };

    void reset_msg_counters() {
        prl_rx.msg_id_stored = -1;
        prl_tx.msg_id_counter = 0;
    };
    void reset_revision() { revision = PD_REVISION::REV30; };

    bool is_running() { return true; };
    bool is_busy() { return false; };
    void send_ctrl_msg(PD_CTRL_MSGT::Type msgt);
    void send_data_msg(PD_DATA_MSGT::Type msgt);
    void send_ext_msg(PD_EXT_MSGT::Type msgt);

    // Mark TX chunk for sending (+ cleanup status flags from prev operations)
    void tx_enquire_chunk();
    // Fill revision / message id fielts and send packet to TCPC driver
    void tcpc_enquire_msg();

    // Helper to report PE errors outside of FSM.
    void report_pending_error();

    // Disable unexpected use
    PRL() = delete;
    PRL(const PRL&) = delete;
    PRL& operator=(const PRL&) = delete;

    enum { LS_DISABLED, LS_INIT, LS_WORKING } local_state = LS_DISABLED;
    AtomicBits<PRL_FLAG::FLAGS_COUNT> flags{};

    // In full PD stack we should keep separate revision for all SOP*.
    // But in sink we talk with charger only, and single revision for `SOP`
    // is enough.
    PD_REVISION::Type revision{PD_REVISION::REV30};

    Port& port;
    Sink& sink;
    IDriver& tcpc;

    PRL_Tx prl_tx;
    PRL_Rx prl_rx;
    PRL_HR prl_hr;
    PRL_RCH prl_rch;
    PRL_TCH prl_tch;

    PRL_EventListener prl_event_listener;
};

} // namespace pd
