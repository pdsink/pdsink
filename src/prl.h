#pragma once

#include "data_objects.h"
#include "idriver.h"
#include "utils/atomic_bits.h"

#include <etl/cyclic_value.h>
#include <etl/fsm.h>

namespace pd {

class Sink;
class PRL;

enum class PRL_ERROR {
    // Spec has too poor description of PE reactions on errors. Let's keep all
    // details in report, to have more flexibility in error handler.

    RCH_BAD_SEQUENCE,       // wrong input chunk (for chunked messages)
    RCH_SEND_FAIL,          // failed to request next chunk (no GoodCRC)
    RCH_SEQUENCE_DISCARDED, // new message interrupted sequence
    RCH_SEQUENCE_TIMEOUT,   // no response for chunk request

    TCH_ENQUIRE_DISCARDED,  // RCH busy, TCH can't accept new message
    TCH_BAD_SEQUENCE,
    TCH_SEND_FAIL,
    TCH_DISCARDED,
    TCH_SEQUENCE_TIMEOUT,
};

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
    void copy_chunk_data();
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

class TcpcEventHandler : public etl::message_router<TcpcEventHandler, MsgTcpcHardReset, MsgTcpcWakeup, MsgTcpcTransmitStatus> {
public:
    TcpcEventHandler(PRL& prl);
    void on_receive(const MsgTcpcHardReset& msg);
    void on_receive(const MsgTcpcWakeup& msg);
    void on_receive(const MsgTcpcTransmitStatus& msg);
    void on_receive(const MsgTimerEvent& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    PRL& prl;
};

class PRL {
public:
    PRL(Sink& sink, IDriver& tcpc);
    void dispatch(const MsgPdEvents& events, const bool pd_enabled);
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

    // PE notifications handlers
    void on_soft_reset_from_pe();
    void on_hard_reset_from_pe();
    void on_pe_hard_reset_done();

    // Helper to report PE errors outside of FSM.
    void report_pending_error();

    // Disable unexpected use
    PRL() = delete;
    PRL(const PRL&) = delete;
    PRL& operator=(const PRL&) = delete;

    enum { LS_DISABLED, LS_INIT, LS_WORKING } local_state = LS_DISABLED;
    AtomicBits<PRL_FLAG::FLAGS_COUNT> flags{};

    PD_MSG rx_emsg;
    PD_MSG tx_emsg;

    // Internal buffers to construct/split messages
    PKT_INFO rx_chunk;
    PKT_INFO tx_chunk;

    // In full PD stack we should keep separate revision for all SOP*.
    // But in sink we talk with charger only, and single revision for `SOP`
    // is enough.
    PD_REVISION::Type revision{PD_REVISION::REV30};

    Sink& sink;
    IDriver& tcpc;

    TcpcEventHandler tcpc_event_handler{*this};
    PRL_Tx prl_tx;
    PRL_Rx prl_rx;
    PRL_HR prl_hr;
    PRL_RCH prl_rch;
    PRL_TCH prl_tch;
};

} // namespace pd
