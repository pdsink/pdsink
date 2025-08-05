#include <etl/array.h>

#include "common_macros.h"
#include "idriver.h"
#include "pd_log.h"
#include "port.h"
#include "prl.h"
#include "utils/etl_state_pack.h"

namespace pd {

// [rev3.2] 6.12.3 List of Protocol Layer States
// Table 6.75 Protocol Layer States

// Chunked receive
enum PRL_RCH_State {
    RCH_Wait_For_Message_From_Protocol_Layer,
    RCH_Pass_Up_Message,
    RCH_Processing_Extended_Message,
    RCH_Requesting_Chunk,
    RCH_Waiting_Chunk,
    RCH_Report_Error,
};

namespace {
    constexpr auto prl_rch_state_to_desc(int state) -> const char* {
        switch (state) {
            case RCH_Wait_For_Message_From_Protocol_Layer: return "RCH_Wait_For_Message_From_Protocol_Layer";
            case RCH_Pass_Up_Message: return "RCH_Pass_Up_Message";
            case RCH_Processing_Extended_Message: return "RCH_Processing_Extended_Message";
            case RCH_Requesting_Chunk: return "RCH_Requesting_Chunk";
            case RCH_Waiting_Chunk: return "RCH_Waiting_Chunk";
            case RCH_Report_Error: return "RCH_Report_Error";
            default: return "Unknown PRL_RCH state";
        }
    }
} // namespace

// Chunked transmit
enum PRL_TCH_State {
    TCH_Wait_For_Message_Request_From_Policy_Engine,
    TCH_Pass_Down_Message,
    TCH_Wait_For_Transmision_Complete,
    TCH_Message_Sent,
    TCH_Prepare_To_Send_Chunked_Message,
    TCH_Construct_Chunked_Message,
    TCH_Sending_Chunked_Message,
    TCH_Wait_Chunk_Request,
    TCH_Message_Received,
    TCH_Report_Error,
};

namespace {
    constexpr auto prl_tch_state_to_desc(int state) -> const char* {
        switch (state) {
            case TCH_Wait_For_Message_Request_From_Policy_Engine: return "TCH_Wait_For_Message_Request_From_Policy_Engine";
            case TCH_Pass_Down_Message: return "TCH_Pass_Down_Message";
            case TCH_Wait_For_Transmision_Complete: return "TCH_Wait_For_Transmision_Complete";
            case TCH_Message_Sent: return "TCH_Message_Sent";
            case TCH_Prepare_To_Send_Chunked_Message: return "TCH_Prepare_To_Send_Chunked_Message";
            case TCH_Construct_Chunked_Message: return "TCH_Construct_Chunked_Message";
            case TCH_Sending_Chunked_Message: return "TCH_Sending_Chunked_Message";
            case TCH_Wait_Chunk_Request: return "TCH_Wait_Chunk_Request";
            case TCH_Message_Received: return "TCH_Message_Received";
            case TCH_Report_Error: return "TCH_Report_Error";
            default: return "Unknown PRL_TCH state";
        }
    }
} // namespace

// Message Transmission
enum PRL_Tx_State {
    PRL_Tx_PHY_Layer_Reset,
    PRL_Tx_Wait_for_Message_Request,
    PRL_Tx_Layer_Reset_for_Transmit,
    PRL_Tx_Construct_Message,
    PRL_Tx_Wait_for_PHY_Response,
    PRL_Tx_Match_MessageID,
    PRL_Tx_Message_Sent,
    PRL_Tx_Check_RetryCounter,
    PRL_Tx_Transmission_Error,
    PRL_Tx_Discard_Message,
    PRL_Tx_Snk_Start_of_AMS,
    PRL_Tx_Snk_Pending,
};

namespace {
    constexpr auto prl_tx_state_to_desc(int state) -> const char* {
        switch (state) {
            case PRL_Tx_PHY_Layer_Reset: return "PRL_Tx_PHY_Layer_Reset";
            case PRL_Tx_Wait_for_Message_Request: return "PRL_Tx_Wait_for_Message_Request";
            case PRL_Tx_Layer_Reset_for_Transmit: return "PRL_Tx_Layer_Reset_for_Transmit";
            case PRL_Tx_Construct_Message: return "PRL_Tx_Construct_Message";
            case PRL_Tx_Wait_for_PHY_Response: return "PRL_Tx_Wait_for_PHY_Response";
            case PRL_Tx_Match_MessageID: return "PRL_Tx_Match_MessageID";
            case PRL_Tx_Message_Sent: return "PRL_Tx_Message_Sent";
            case PRL_Tx_Check_RetryCounter: return "PRL_Tx_Check_RetryCounter";
            case PRL_Tx_Transmission_Error: return "PRL_Tx_Transmission_Error";
            case PRL_Tx_Discard_Message: return "PRL_Tx_Discard_Message";
            case PRL_Tx_Snk_Start_of_AMS: return "PRL_Tx_Snk_Start_of_AMS";
            case PRL_Tx_Snk_Pending: return "PRL_Tx_Snk_Pending";
            default: return "Unknown PRL_Tx state";
        }
    }
} // namespace

// Message Reception
enum PRL_Rx_State {
    PRL_Rx_Wait_for_PHY_Message,
    PRL_Rx_Layer_Reset_for_Receive,
    PRL_Rx_Send_GoodCRC,
    PRL_Rx_Check_MessageID,
    PRL_Rx_Store_MessageID,
};

namespace {
    constexpr auto prl_rx_state_to_desc(int state) -> const char* {
        switch (state) {
            case PRL_Rx_Wait_for_PHY_Message: return "PRL_Rx_Wait_for_PHY_Message";
            case PRL_Rx_Layer_Reset_for_Receive: return "PRL_Rx_Layer_Reset_for_Receive";
            case PRL_Rx_Send_GoodCRC: return "PRL_Rx_Send_GoodCRC";
            case PRL_Rx_Check_MessageID: return "PRL_Rx_Check_MessageID";
            case PRL_Rx_Store_MessageID: return "PRL_Rx_Store_MessageID";
            default: return "Unknown PRL_Rx state";
        }
    }
} // namespace

// Hard Reset
enum PRL_HR_State {
    PRL_HR_IDLE,
    PRL_HR_Reset_Layer,
    PRL_HR_Indicate_Hard_Reset,
    PRL_HR_Request_Hard_Reset,
    PRL_HR_Wait_for_PHY_Hard_Reset_Complete,
    PRL_HR_PHY_Hard_Reset_Requested,
    PRL_HR_Wait_for_PE_Hard_Reset_Complete,
    PRL_HR_PE_Hard_Reset_Complete,
};

namespace {
    constexpr auto prl_hr_state_to_desc(int state) -> const char* {
        switch (state) {
            case PRL_HR_IDLE: return "PRL_HR_IDLE";
            case PRL_HR_Reset_Layer: return "PRL_HR_Reset_Layer";
            case PRL_HR_Indicate_Hard_Reset: return "PRL_HR_Indicate_Hard_Reset";
            case PRL_HR_Request_Hard_Reset: return "PRL_HR_Request_Hard_Reset";
            case PRL_HR_Wait_for_PHY_Hard_Reset_Complete: return "PRL_HR_Wait_for_PHY_Hard_Reset_Complete";
            case PRL_HR_PHY_Hard_Reset_Requested: return "PRL_HR_PHY_Hard_Reset_Requested";
            case PRL_HR_Wait_for_PE_Hard_Reset_Complete: return "PRL_HR_Wait_for_PE_Hard_Reset_Complete";
            case PRL_HR_PE_Hard_Reset_Complete: return "PRL_HR_PE_Hard_Reset_Complete";
            default: return "Unknown PRL_HR state";
        }
    }
} // namespace


//
// Macros to quick-create common methods
//
#define ON_ENTER_STATE_DEFAULT \
auto on_enter_state() -> etl::fsm_state_id_t override { \
    get_fsm_context().log_state(); \
    return No_State_Change; \
}

#define ON_UNKNOWN_EVENT_DEFAULT \
auto on_event_unknown(const etl::imessage&) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}

#define ON_TRANSIT_TO \
auto on_event(const MsgTransitTo& event) -> etl::fsm_state_id_t { \
    return event.state_id; \
}

#define ON_EVENT_NOTHING \
auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.1.2 Chunked Rx State Diagram

class RCH_Wait_For_Message_From_Protocol_Layer_State : public etl::fsm_state<PRL_RCH, RCH_Wait_For_Message_From_Protocol_Layer_State, RCH_Wait_For_Message_From_Protocol_Layer, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    // Spec requires to clear extended message buffer (rx_emsg) on enter,
    // but we do that on first chunk instead. Because buffer is shared with PE.

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;
        auto& port = prl.port;

        if (port.prl_rch_flags.test_and_clear(RCH_FLAG::RX_ENQUEUED)) {
            // Copy header to output struct
            port.rx_emsg.header = port.rx_chunk.header;

            if (port.rx_chunk.header.extended) {
                PD_EXT_HEADER ehdr{port.rx_chunk.read16(0)};

                if (ehdr.chunked) {
                    // Spec says clear vars below in RCH_Processing_Extended_Message
                    // on first chunk, but this place looks more obvious
                    port.rch_chunk_number_expected = 0;
                    port.rx_emsg.clear();
                    port.rx_emsg.header = port.rx_chunk.header;
                    return RCH_Processing_Extended_Message;
                }

                // Unchunked ext messages are not supported
                port.rch_error = PRL_ERROR::RCH_BAD_SEQUENCE;
                return RCH_Report_Error;
            }

            // Non-extended message
            port.rx_emsg = port.rx_chunk;
            port.rx_emsg.resize_by_data_obj_count();
            return RCH_Pass_Up_Message;
        }
        return No_State_Change;
    }
};

class RCH_Pass_Up_Message_State : public etl::fsm_state<PRL_RCH, RCH_Pass_Up_Message_State, RCH_Pass_Up_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        rch.log_state();

        rch.prl.port.notify_pe(MsgToPe_PrlMessageReceived{});
        return RCH_Wait_For_Message_From_Protocol_Layer;
    }
};

class RCH_Processing_Extended_Message_State : public etl::fsm_state<PRL_RCH, RCH_Processing_Extended_Message_State, RCH_Processing_Extended_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;
        rch.log_state();


        PD_EXT_HEADER ehdr{port.rx_chunk.read16(0)};

        // Data integrity check
        if ((ehdr.chunk_number != port.rch_chunk_number_expected) ||
            (ehdr.chunk_number >= MaxChunksPerMsg) ||
            (ehdr.data_size > MaxExtendedMsgLen) ||
            (ehdr.request_chunk != 0) ||
            (ehdr.chunked != 1))
        {
            port.rch_error = PRL_ERROR::RCH_BAD_SEQUENCE;
            return RCH_Report_Error;
        }

        // Copy as much as possible (without ext header),
        // until desired size reached.
        port.rx_emsg.append_from(port.rx_chunk, 2, port.rx_chunk.data_size());
        port.rch_chunk_number_expected++;

        if (port.rx_emsg.data_size() >= ehdr.data_size) {
            port.rx_emsg.get_data().resize(ehdr.data_size);
            return RCH_Pass_Up_Message;
        }
        return RCH_Requesting_Chunk;
    }
};

class RCH_Requesting_Chunk_State : public etl::fsm_state<PRL_RCH, RCH_Requesting_Chunk_State, RCH_Requesting_Chunk, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;
        rch.log_state();

        // Block PE timeout timer for multichunk responses, it should not fail
        port.timers.stop(PD_TIMEOUT::tSenderResponse);

        PD_HEADER hdr{0};
        hdr.message_type = port.rx_emsg.header.message_type;
        hdr.data_obj_count = 1;
        hdr.extended = 1;

        PD_EXT_HEADER ehdr{0};
        ehdr.request_chunk = 1;
        ehdr.chunk_number = port.rch_chunk_number_expected;
        ehdr.chunked = 1;

        auto& chunk = port.tx_chunk;
        chunk.clear();
        chunk.header = hdr;
        chunk.append16(ehdr.raw_value);
        chunk.append16(0); // Placeholder, align to 32 bit (data object size)

        // Mark chunk for send
        rch.prl.prl_tx_enquire_chunk();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            return RCH_Waiting_Chunk;
        }

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            port.rch_error = PRL_ERROR::RCH_SEND_FAIL;
            return RCH_Report_Error;
        }

        // Catch simultaneous RX/TX + discard. Suppose tx was successful,
        // and decide what really happened at next states.
        // Here we can have new message from PRL_RX before PRL_TX called
        if (port.prl_rch_flags.test(RCH_FLAG::RX_ENQUEUED)) {
            return RCH_Waiting_Chunk;
        }

        return No_State_Change;
    }
};

class RCH_Waiting_Chunk_State : public etl::fsm_state<PRL_RCH, RCH_Waiting_Chunk_State, RCH_Waiting_Chunk, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;
        rch.log_state();

        port.timers.start(PD_TIMEOUT::tChunkSenderResponse);
        port.timers.start(PD_TIMEOUT::tSenderResponse);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;

        if (port.prl_rch_flags.test(RCH_FLAG::RX_ENQUEUED)) {
            // Spec requires to inform PE immediately about new message on
            // wrong sequence, prior returning to
            // RCH_Wait_For_Message_From_Protocol_Layer.
            // But we can safely land only unchunked messages this way.

            // NOTE: if unchunked ext msg ever supported, filter here too.
            if (port.rx_chunk.header.extended == 0) {
                port.rch_error = PRL_ERROR::RCH_SEQUENCE_DISCARDED;
                return RCH_Report_Error;
            }

            // Now disable message forward in error reporter, and continue
            // checks in next state. Everything not matched will be
            // a pure error (without message forward).
            port.prl_rch_flags.clear(RCH_FLAG::RX_ENQUEUED);
            return RCH_Processing_Extended_Message;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tChunkSenderResponse)) {
            port.rch_error = PRL_ERROR::RCH_SEQUENCE_TIMEOUT;
            return RCH_Report_Error;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().prl.port;
        port.timers.stop(PD_TIMEOUT::tChunkSenderResponse);
    }
};

class RCH_Report_Error_State : public etl::fsm_state<PRL_RCH, RCH_Report_Error_State, RCH_Report_Error, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& port = rch.prl.port;

        if (port.prl_rch_flags.test_and_clear(RCH_FLAG::RX_ENQUEUED)) {
            port.rx_emsg = port.rx_chunk;
            port.rx_emsg.resize_by_data_obj_count();
            port.notify_pe(MsgToPe_PrlMessageReceived{});
        }

        port.notify_pe(MsgToPe_PrlReportError{port.rch_error});
        return RCH_Wait_For_Message_From_Protocol_Layer;
    }
};


////////////////////////////////////////////////////////////////////////////////

class TCH_Wait_For_Message_Request_From_Policy_Engine_State : public etl::fsm_state<PRL_TCH, TCH_Wait_For_Message_Request_From_Policy_Engine_State, TCH_Wait_For_Message_Request_From_Policy_Engine, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;

        // [rev3.2] 6.12.2.1.3 Chunked Tx State Diagram
        // Any Message Received and not in state TCH_Wait_Chunk_Request
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }

        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::MSG_FROM_PE_ENQUEUED)) {
            if (tch.prl.prl_rch.get_state_id() != RCH_Wait_For_Message_From_Protocol_Layer) {
                //
                // This may happen, when
                // - PRL was NOT busy
                // - PE started DPM request
                // - Got message from partner and RCH started to process it
                //
                // Spec says, reaction depends on implementation of optional ABORT flag.
                // Since absolutely no ideas how to use that ABORT flag in real world,
                // it's not implemented. So, according to spec, we just discard
                // PE request and stay in the same state.
                //
                // In context of RCH/TCH transparency for PE, this behaviour looks
                // more consistent than error reporting (the same as discarding TX by RX).
                //
                port.notify_pe(MsgToPe_PrlReportDiscard{});
                return No_State_Change;
            }

            if (port.tx_emsg.header.extended) {
                return TCH_Prepare_To_Send_Chunked_Message;
            }
            return TCH_Pass_Down_Message;
        }
        return No_State_Change;
    }
};

class TCH_Pass_Down_Message_State : public etl::fsm_state<PRL_TCH, TCH_Pass_Down_Message_State, TCH_Pass_Down_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        // Copy data to chunk & fill data object count
        port.tx_chunk = port.tx_emsg;
        port.tx_chunk.header.data_obj_count = (port.tx_emsg.data_size() + 3) >> 2;

        tch.prl.prl_tx_enquire_chunk();
        return TCH_Wait_For_Transmision_Complete;
    }
};

class TCH_Wait_For_Transmision_Complete_State : public etl::fsm_state<PRL_TCH, TCH_Wait_For_Transmision_Complete_State, TCH_Wait_For_Transmision_Complete, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            return TCH_Message_Sent;
        }

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            port.tch_error = PRL_ERROR::TCH_SEND_FAIL;
            return TCH_Report_Error;
        }

        // Catch message discard (indirectly). This happens when new message
        // routed to TCH.
        //
        // Ignore case when driver status is TCPC_TRANSMIT_STATUS::SUCCESS.
        // That means driver did transfer, but PRL_TX not been called yet.
        // This is possible, because TX and RX events can arrive in the same
        // time. In this case just wait until PRL_TX called.

        if (port.tcpc_tx_status.load() == TCPC_TRANSMIT_STATUS::SUCCEEDED) {
            port.wakeup(); // Probably not needed, but just in case.
            return No_State_Change;
        }

        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            // At this point, discard already reported by PRL_TX
            return TCH_Message_Received;
        }

        return No_State_Change;
    }
};

class TCH_Message_Sent_State : public etl::fsm_state<PRL_TCH, TCH_Message_Sent_State, TCH_Message_Sent, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        port.notify_pe(MsgToPe_PrlMessageSent{});

        // [rev3.2] 6.12.2.1.3 Chunked Tx State Diagram
        // Any Message Received and not in state TCH_Wait_Chunk_Request
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }
        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }
};

class TCH_Prepare_To_Send_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Prepare_To_Send_Chunked_Message_State, TCH_Prepare_To_Send_Chunked_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        port.tch_chunk_number_to_send = 0;
        return TCH_Construct_Chunked_Message;
    }
};

class TCH_Construct_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Construct_Chunked_Message_State, TCH_Construct_Chunked_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        int tail_len = port.tx_emsg.data_size() - port.tch_chunk_number_to_send * MaxExtendedMsgChunkLen;
        int chunk_data_len = etl::min(tail_len, MaxExtendedMsgChunkLen);

        PD_EXT_HEADER ehdr{0};
        ehdr.data_size = port.tx_emsg.data_size();
        ehdr.chunk_number = port.tch_chunk_number_to_send;
        ehdr.chunked = 1;

        port.tx_chunk.clear();
        port.tx_chunk.append16(ehdr.raw_value);
        auto offset = port.tch_chunk_number_to_send * MaxExtendedMsgChunkLen;
        port.tx_chunk.append_from(port.tx_emsg, offset, offset + chunk_data_len);

        port.tx_chunk.header = port.tx_emsg.header;
        port.tx_chunk.header.data_obj_count = (port.tx_chunk.data_size() + 3) >> 2;

        tch.prl.prl_tx_enquire_chunk();
        return TCH_Sending_Chunked_Message;
    }
};

class TCH_Sending_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Sending_Chunked_Message_State, TCH_Sending_Chunked_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            port.tch_error = PRL_ERROR::TCH_SEND_FAIL;
            return TCH_Report_Error;
        }

        // Calculate max possible bytes been sent, if all chunks are of max size
        uint32_t max_bytes = (port.tch_chunk_number_to_send + 1) * MaxExtendedMsgChunkLen;

        // Reached msg size => last chunk sent. LLand it without error,
        // even if new message received.
        if ((max_bytes >= port.tx_emsg.data_size()) &&
            port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED))
        {
            return TCH_Message_Sent;
        }

        // Handle discard (RX not in TCH_Wait_Chunk_Request). PE already
        // notified about discard by RX/TX at this moment.
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }

        // Succeeded but, not last chunk
        if ((max_bytes < port.tx_emsg.data_size()) &&
            port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED))
        {
            return TCH_Wait_Chunk_Request;
        }

        return No_State_Change;
    }
};

class TCH_Wait_Chunk_Request_State : public etl::fsm_state<PRL_TCH, TCH_Wait_Chunk_Request_State, TCH_Wait_Chunk_Request, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        port.tch_chunk_number_to_send++;
        port.timers.start(PD_TIMEOUT::tChunkSenderRequest);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;

        if (port.prl_tch_flags.test(TCH_FLAG::CHUNK_FROM_RX)) {
            if (port.rx_emsg.header.extended) {
                PD_EXT_HEADER ehdr{port.rx_chunk.read16(0)};

                if (ehdr.request_chunk == 1) {
                    if (ehdr.chunk_number == port.tch_chunk_number_to_send) {
                        port.prl_tch_flags.clear(TCH_FLAG::CHUNK_FROM_RX);
                        return TCH_Construct_Chunked_Message;
                    }

                    port.prl_tch_flags.clear(TCH_FLAG::CHUNK_FROM_RX);
                    port.tch_error = PRL_ERROR::TCH_BAD_SEQUENCE;
                    return TCH_Report_Error;
                }
            }

            // [rev3.2] 6.12.2.1.3.8 TCH_Wait_Chunk_Request State
            // Any other Message than Chunk Request is received.

            // TODO: It's not clear why error/discard is not reported
            // when chunked sending was interrupted instead of consuming next
            // chunks.
            return TCH_Message_Received;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tChunkSenderRequest)) {
            port.tch_error = PRL_ERROR::TCH_SEQUENCE_TIMEOUT;
            return TCH_Report_Error;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().prl.port;
        port.timers.stop(PD_TIMEOUT::tChunkSenderRequest);
    }
};

class TCH_Message_Received_State : public etl::fsm_state<PRL_TCH, TCH_Message_Received_State, TCH_Message_Received, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        // Forward PRL_RX message to RCH
        port.prl_rch_flags.set(RCH_FLAG::RX_ENQUEUED);
        port.wakeup();

        // Drop incoming TCH request from PE if any
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::MSG_FROM_PE_ENQUEUED)) {
            port.notify_pe(MsgToPe_PrlReportDiscard{});
        }

        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }
};

class TCH_Report_Error_State : public etl::fsm_state<PRL_TCH, TCH_Report_Error_State, TCH_Report_Error, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& port = tch.prl.port;
        tch.log_state();

        port.notify_pe(MsgToPe_PrlReportError{port.tch_error});

        if (port.prl_tch_flags.test(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }
        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }
};


////////////////////////////////////////////////////////////////////////////////
// Note, this is low level layer for packet rx/tx.
//
// - Only discards are reported to PE from here
// - Success/errors are forwarded to RCH/TCH via flags.
// - Some room is reserved for CRC processing, to stay close to spec. But
//   but currently only branches for hardware-supported GoodCRC are actual.
//   This should be revisited and cleaned if software CRC support is not actual.

class PRL_Tx_PHY_Layer_Reset_State : public etl::fsm_state<PRL_Tx, PRL_Tx_PHY_Layer_Reset_State, PRL_Tx_PHY_Layer_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Technically, we should call set_rx_enable(true). But since call is
        // async - postpone it for next state, to have vars init coordinated.
        port.prl_tx_flags.set(PRL_TX_FLAG::IS_FROM_LAYER_RESET);
        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Wait_for_Message_Request_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Wait_for_Message_Request_State, PRL_Tx_Wait_for_Message_Request, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
        port.tx_retry_counter = 0;

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::IS_FROM_LAYER_RESET)) {
            prl_tx.prl.tcpc.req_rx_enable(true);
        }

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;
        auto& port = prl.port;

        if (prl.tcpc.is_rx_enable_done()) { return No_State_Change; }

        // For first AMS message need to wait SinkTxOK CC level
        if (!port.is_ams_active()) {
            port.prl_tx_flags.clear(PRL_TX_FLAG::START_OF_AMS_DETECTED);
        } else {
            if (!port.prl_tx_flags.test(PRL_TX_FLAG::START_OF_AMS_DETECTED)) {
                port.prl_tx_flags.set(PRL_TX_FLAG::START_OF_AMS_DETECTED);
                return PRL_Tx_Snk_Start_of_AMS;
            }
        }

        // For non-AMS messages, or after first AMS message
        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            if (port.tx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
                return PRL_Tx_Layer_Reset_for_Transmit;
            }
            return PRL_Tx_Construct_Message;
        }

        return No_State_Change;
    }
};

class PRL_Tx_Layer_Reset_for_Transmit_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Layer_Reset_for_Transmit_State, PRL_Tx_Layer_Reset_for_Transmit, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        auto& prl = prl_tx.prl;
        prl_tx.log_state();

        // Note, spec says to reset only `msg_id_counter` here, and reset
        // `msg_id_stored` via RX state change. But etl::fsm does not re-run
        // `on_enter` if we come from current state to itself.
        // So, reset both here.
        prl.reset_msg_counters();
        // This will not make sense, because we do not send GoodCRC in software,
        // and every input packet causes returns to initial state immediately.
        // But this is left for consistency with the spec.
        prl.prl_rx.receive(MsgTransitTo(PRL_Rx_Wait_for_PHY_Message));

        return PRL_Tx_Construct_Message;
    }
};

class PRL_Tx_Construct_Message_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Construct_Message_State, PRL_Tx_Construct_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        prl_tx.prl.tcpc_enquire_msg();
        return No_State_Change;
    }
};

// Here we wait for "GoodCRC" or failure.
class PRL_Tx_Wait_for_PHY_Response_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Wait_for_PHY_Response_State, PRL_Tx_Wait_for_PHY_Response, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Timer should be used ONLY when hardware confirmation not supported
        // if (!prl_tx.prl.tcpc.get_hw_features().tx_goodcrc_receive) {
        //    prl_tx.prl.port.timers.start(PD_TIMEOUT::tReceive);
        // }
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().prl.port;

        auto status = port.tcpc_tx_status.load();

        if (status == TCPC_TRANSMIT_STATUS::SUCCEEDED) {
            return PRL_Tx_Match_MessageID;
        }

        //
        // NOTE: do NOT check TCPC_TRANSMIT_STATUS::DISCARDED.
        // Rely on teleport to PRL_Tx_Discard_Message by prl_rx,
        // to have new msg + discard in sync.
        //

        if (status == TCPC_TRANSMIT_STATUS::FAILED) {
            return PRL_Tx_Check_RetryCounter;
        }

        // Actual only for software CRC processing
        // if (port.timers.is_expired(PD_TIMEOUT::tReceive)) {
        //    return PRL_Tx_Check_RetryCounter;
        // }

        return No_State_Change;
    }

    // auto on_exit_state() -> void {
    //    auto& prl_tx = get_fsm_context();
    //    prl_tx.prl.port.timers.stop(PD_TIMEOUT::tReceive);
    // }
};

class PRL_Tx_Match_MessageID_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Match_MessageID_State, PRL_Tx_Match_MessageID, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Since msg id match currently embedded in transfer success status,
        // just forward to next state
        return PRL_Tx_Message_Sent;
    }
};

class PRL_Tx_Message_Sent_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Message_Sent_State, PRL_Tx_Message_Sent, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tx_msg_id_counter++;
        port.prl_tx_flags.set(PRL_TX_FLAG::TX_COMPLETED);

        // Ensure one more loop run, to invoke RCH/TCH after PRL_TX completed
        // TODO: can be removed if RCH/TCH FSMs are invoked after PRL_TX.
        port.wakeup();

        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Check_RetryCounter_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Check_RetryCounter_State, PRL_Tx_Check_RetryCounter, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Retries are NOT used:
        //
        // - for Cable Plug
        // - for Extended Message with data size > MaxExtendedMsgLegacyLen that
        //   has not been chunked
        //
        // Since we are Sink-only, without unchunked ext msg support - no extra
        // checks needed. Always use retries.

        if (prl_tx.prl.tcpc.get_hw_features().tx_retransmit) {
            // TODO: consider removing this feature.

            // Don't try retransmit if supported by hardware.
            return PRL_Tx_Transmission_Error;
        }

        port.tx_retry_counter++;

        // TODO: check if retries count should depend on negotiated revision (2 or 3)
        if (port.tx_retry_counter > 2 /* nRetryCount */) {
            return PRL_Tx_Transmission_Error;
        }
        return PRL_Tx_Construct_Message;
    }
};

class PRL_Tx_Transmission_Error_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Transmission_Error_State, PRL_Tx_Transmission_Error, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tx_msg_id_counter++;
        // Don't report PE about error here, allow RCH/TCH to handle & care.
        port.prl_tx_flags.set(PRL_TX_FLAG::TX_ERROR);

        // Ensure one more loop run, to invoke RCH/TCH after PRL_TX completed
        // TODO: can be removed if RCH/TCH FSMs are invoked after PRL_TX.
        port.wakeup();

        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Discard_Message_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Discard_Message_State, PRL_Tx_Discard_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;

        // Do discard, if any TX chunk processing:
        // - input queued to send
        // - passed to driver, and sending in progress
        // - if driver signaled discard
        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED) ||
            port.tcpc_tx_status.load() == TCPC_TRANSMIT_STATUS::WAITING)
        {
            port.tx_msg_id_counter++;
            port.notify_pe(MsgToPe_PrlReportDiscard{});
        }
        return PRL_Tx_PHY_Layer_Reset;
    }
};

class PRL_Tx_Snk_Start_of_AMS_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Snk_Start_of_AMS_State, PRL_Tx_Snk_Start_of_AMS, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Reuse existing event to switch state, if condition satisfied
        if (port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().prl.port;

        if (port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }
};

class PRL_Tx_Snk_Pending_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Snk_Pending_State, PRL_Tx_Snk_Pending, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Soft reset passed without delay
        if (port.tx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
            port.prl_tx_flags.clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
            return PRL_Tx_Layer_Reset_for_Transmit;
        }

        prl_tx.prl.tcpc.req_active_cc();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;
        auto& port = prl.port;
        auto& tcpc = prl.tcpc;

        // Wait until CC fetch completes
        TCPC_CC_LEVEL::Type cc_level;
        if (!tcpc.try_active_cc_result(cc_level)) { return No_State_Change; }

        // Wait SinkTxOK before sending first AMS message
        if (cc_level == TCPC_CC_LEVEL::SinkTxOK) {
            port.prl_tx_flags.clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
            return PRL_Tx_Construct_Message;
        }

        if (port.timers.is_disabled(PD_TIMEOUT::tActiveCcPollingDebounce)) {
            port.timers.start(PD_TIMEOUT::tActiveCcPollingDebounce);
        }

        if (port.timers.is_expired(PD_TIMEOUT::tActiveCcPollingDebounce)) {
            port.timers.stop(PD_TIMEOUT::tActiveCcPollingDebounce);
            prl.tcpc.req_active_cc();
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().prl.port;
        port.timers.stop(PD_TIMEOUT::tActiveCcPollingDebounce);
    }
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.3 Protocol Layer Message Reception

class PRL_Rx_Wait_for_PHY_Message_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Wait_for_PHY_Message_State, PRL_Rx_Wait_for_PHY_Message, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;
        auto& port = prl.port;

        if (port.prl_rch_flags.test(RCH_FLAG::RX_ENQUEUED)) {
            // In theory, we can have pending packet in RCH, re-routed by
            // discard. Postpone processing of new one to next cycle, to allow
            // RCH to finish.
            //
            // This is not expected to happen, because we do multiple RCH/TCH
            // calls.
            port.wakeup();
            return No_State_Change;
        }

        if (!prl.tcpc.fetch_rx_data()) { return No_State_Change; }

        if (port.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
            return PRL_Rx_Layer_Reset_for_Receive;
        }

        return PRL_Rx_Send_GoodCRC;
    }
};

class PRL_Rx_Layer_Reset_for_Receive_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Layer_Reset_for_Receive_State, PRL_Rx_Layer_Reset_for_Receive, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        auto& prl = prl_rx.prl;
        auto& port = prl.port;
        prl_rx.log_state();

        // Similar to init, but skip RX and (?) revision clear.
        port.prl_tx_flags.clear_all();
        port.prl_rch_flags.clear_all();
        port.prl_tch_flags.clear_all();

        prl.reset_msg_counters();

        prl.prl_rch.reset();
        prl.prl_rch.start();

        prl.prl_tch.reset();
        prl.prl_tch.start();

        prl.prl_tx.reset();
        prl.prl_tx.start();

        port.notify_pe(MsgToPe_PrlSoftResetFromPartner{});
        return PRL_Rx_Send_GoodCRC;
    }
};

class PRL_Rx_Send_GoodCRC_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Send_GoodCRC_State, PRL_Rx_Send_GoodCRC, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    // All modern hardware send CRC automatically. This state exists
    // to match spec and for potential extensions.
    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        prl_rx.log_state();
        return PRL_Rx_Check_MessageID;
    }
};

class PRL_Rx_Check_MessageID_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Check_MessageID_State, PRL_Rx_Check_MessageID, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        auto& port = prl_rx.prl.port;
        prl_rx.log_state();

        if (port.rx_msg_id_stored == port.rx_chunk.header.message_id) {
            // Ignore duplicated
            return PRL_Rx_Wait_for_PHY_Message;
        }
        return PRL_Rx_Store_MessageID;
    }
};

class PRL_Rx_Store_MessageID_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Store_MessageID_State, PRL_Rx_Store_MessageID, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        auto& port = prl_rx.prl.port;
        auto& prl = prl_rx.prl;
        prl_rx.log_state();

        port.rx_msg_id_stored = port.rx_chunk.header.message_id;

        // Rev 3.2 says ping is deprecated => Ignore it completely
        // (it should not discard, affect chunking and so on).
        if (port.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Ping)) { return PRL_Rx_Wait_for_PHY_Message; }

        // Discard TX if:
        //
        // - new data enquired (but not sent yet)
        // - sending in progress
        // - failed/discarded
        //
        // Don't discard if sending succeeded. Let it finish as usual, because
        // this status can arrive together with new incoming message.
        //
        auto status = port.tcpc_tx_status.load();
        if ((status != TCPC_TRANSMIT_STATUS::UNSET && status != TCPC_TRANSMIT_STATUS::SUCCEEDED) ||
            port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED))
        {
            prl.prl_tx.receive(MsgTransitTo(PRL_Tx_Discard_Message));
        }

        // [rev3.2] 6.12.2.1.4 Chunked Message Router State Diagram
        //
        // Route message to RCH/TCH. Since RTR has no stored states, it is
        // more simple to embed it's logic here.

        // Spec describes TCH doing chunking as
        // "Not in TCH_Wait_For_Message_Request_From_Policy_Engine state".
        // But PE sending requests are not executed immediately, those just
        // rise flag. So, having that flag set means "not waiting" too.
        // Because TCH will leave waiting state on nearest call.
        if (port.prl_tch_flags.test(TCH_FLAG::MSG_FROM_PE_ENQUEUED) ||
            prl_rx.prl.prl_tch.get_state_id() != TCH_Wait_For_Message_Request_From_Policy_Engine)
        {
            // TCH does chunking => route to it
            port.prl_tch_flags.set(TCH_FLAG::CHUNK_FROM_RX);
        } else {
            // No TCH chunking => route to RCH
            port.prl_rch_flags.set(RCH_FLAG::RX_ENQUEUED);
        }

        // Return back to waiting.
        return PRL_Rx_Wait_for_PHY_Message;

    }
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.4 Hard Reset operation

class PRL_HR_IDLE_State : public etl::fsm_state<PRL_HR, PRL_HR_IDLE_State, PRL_HR_IDLE, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().prl.port;

        if (port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER) ||
            port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PE))
        {
            return PRL_HR_Reset_Layer;
        }
        return No_State_Change;
    }
};

class PRL_HR_Reset_Layer_State : public etl::fsm_state<PRL_HR, PRL_HR_Reset_Layer_State, PRL_HR_Reset_Layer, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        // First, we don't "reset" anything in this state, because PRL_TX fsm
        // reset will cause enabling RX back. Until PRL_HR returns to initial
        // state, events are not precessed, and that looks safe.

        // Start with RX path disable (and FIFO clear).
        hr.prl.tcpc.req_rx_enable(false);

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        // Wait for TCPC operation complete
        if (prl.tcpc.is_rx_enable_done()) {
            return No_State_Change;
        }

        // Route state, depending on hard reset type requested
        if (prl.port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER)) {
            return PRL_HR_Indicate_Hard_Reset;
        }
        return PRL_HR_Request_Hard_Reset;
    }
};

class PRL_HR_Indicate_Hard_Reset_State : public etl::fsm_state<PRL_HR, PRL_HR_Indicate_Hard_Reset_State, PRL_HR_Indicate_Hard_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.port.notify_pe(MsgToPe_PrlHardResetFromPartner{});
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }
};

class PRL_HR_Request_Hard_Reset_State : public etl::fsm_state<PRL_HR, PRL_HR_Request_Hard_Reset_State, PRL_HR_Request_Hard_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.tcpc.req_hr_send();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        // Wait for TCPC call to complete
        if (prl.tcpc.is_hr_send_done()) {
            return No_State_Change;
        }
        return PRL_HR_Wait_for_PHY_Hard_Reset_Complete;
    }
};

class PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State, PRL_HR_Wait_for_PHY_Hard_Reset_Complete, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        auto& port = hr.prl.port;
        hr.log_state();

        port.timers.start(PD_TIMEOUT::tHardResetComplete);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().prl.port;
        if (port.timers.is_expired(PD_TIMEOUT::tHardResetComplete)) {
            return PRL_HR_PHY_Hard_Reset_Requested;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().prl.port;
        port.timers.stop(PD_TIMEOUT::tHardResetComplete);
    }
};

class PRL_HR_PHY_Hard_Reset_Requested_State : public etl::fsm_state<PRL_HR, PRL_HR_PHY_Hard_Reset_Requested_State, PRL_HR_PHY_Hard_Reset_Requested, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        auto& port = hr.prl.port;
        hr.log_state();

        port.notify_pe(MsgToPe_PrlHardResetSent{});
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }
};

class PRL_HR_Wait_for_PE_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_Wait_for_PE_Hard_Reset_Complete_State, PRL_HR_Wait_for_PE_Hard_Reset_Complete, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().prl.port;

        if (port.prl_hr_flags.test_and_clear(PRL_HR_FLAG::PE_HARD_RESET_COMPLETE)) {
            return PRL_HR_PE_Hard_Reset_Complete;
        }
        return No_State_Change;
    }
};

class PRL_HR_PE_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_PE_Hard_Reset_Complete_State, PRL_HR_PE_Hard_Reset_Complete, MsgSysUpdate, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.init(true);
        hr.prl.port.prl_hr_flags.clear_all();
        return PRL_HR_IDLE;
    }
};


////////////////////////////////////////////////////////////////////////////////


PRL::PRL(Port& port, IDriver& tcpc)
    : port{port},
    tcpc{tcpc},
    prl_tx{*this},
    prl_rx{*this},
    prl_hr{*this},
    prl_rch{*this},
    prl_tch{*this},
    prl_event_listener{*this}
{}

void PRL::setup() {
    port.msgbus.subscribe(prl_event_listener);
}

void PRL::init(bool from_hr_fsm) {
    PRL_LOGI("PRL init");

    // If init called from PRL_HR, don't intrude HR fsm
    if (!from_hr_fsm) {
        port.prl_hr_flags.clear_all();
        prl_hr.reset();
        prl_hr.start();
    }

    // Reset vars before FSMs reset.
    port.prl_tx_flags.clear_all();
    port.prl_rch_flags.clear_all();
    port.prl_tch_flags.clear_all();
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);

    port.timers.stop_range(PD_TIMERS_RANGE::PRL);
    reset_msg_counters();
    reset_revision();

    prl_rch.reset();
    prl_rch.start();

    prl_tch.reset();
    prl_tch.start();

    prl_rx.reset();
    prl_rx.start();

    // Reset TX last, because it does driver call on init.
    prl_tx.reset();
    prl_tx.start();

    // Ensure loop repeat to continue PE States, which wait for PRL run.
    port.wakeup();
}

void PRL::reset_msg_counters() {
    port.rx_msg_id_stored = -1;
    port.tx_msg_id_counter = 0;
}

void PRL::reset_revision() {
    port.revision = PD_REVISION::REV30;
}

void PRL::tcpc_enquire_msg() {
    port.tx_chunk.header.message_id = port.tx_msg_id_counter;
    port.tx_chunk.header.spec_revision = port.revision;

    // Here we should fill power/data roles. But since we are Sink-only UFP,
    // we can just use default values (zeroes).

    // TODO: probably not needed, because the only entry point is
    // `prl_tx_enquire_chunk()`, which clears flags.
    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_COMPLETED);
    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_ERROR);

    // Rearm TCPC TX status.
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::WAITING);

    tcpc.req_transmit();
}

void PRL::prl_tx_enquire_chunk() {
    // TODO: may be used to protect re-calls for old data. Check, if we should
    // leave it or rely on LeapSync.
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);

    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_COMPLETED);
    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_ERROR);
    port.prl_tx_flags.set(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
    // Ensure to call PRL_TX FSM after RCH/TCH
    port.wakeup();
}


////////////////////////////////////////////////////////////////////////////////
// FSMs configs

etl_ext::fsm_state_pack<
    RCH_Wait_For_Message_From_Protocol_Layer_State,
    RCH_Pass_Up_Message_State,
    RCH_Processing_Extended_Message_State,
    RCH_Requesting_Chunk_State,
    RCH_Waiting_Chunk_State,
    RCH_Report_Error_State
> prl_rch_state_list;

PRL_RCH::PRL_RCH(PRL& prl) : etl::fsm(0), prl{prl} {
    set_states(prl_rch_state_list.states, prl_rch_state_list.size);
};

void PRL_RCH::log_state() {
    PRL_LOGI("PRL_RCH state => {}", prl_rch_state_to_desc(get_state_id()));
}

etl_ext::fsm_state_pack<
    TCH_Wait_For_Message_Request_From_Policy_Engine_State,
    TCH_Pass_Down_Message_State,
    TCH_Wait_For_Transmision_Complete_State,
    TCH_Message_Sent_State,
    TCH_Prepare_To_Send_Chunked_Message_State,
    TCH_Construct_Chunked_Message_State,
    TCH_Sending_Chunked_Message_State,
    TCH_Wait_Chunk_Request_State,
    TCH_Message_Received_State,
    TCH_Report_Error_State
> prl_tch_state_list;

PRL_TCH::PRL_TCH(PRL& prl) : etl::fsm(0), prl{prl} {
    set_states(prl_tch_state_list.states, prl_tch_state_list.size);
}

void PRL_TCH::log_state() {
    PRL_LOGI("PRL_TCH state => {}", prl_tch_state_to_desc(get_state_id()));
}

etl_ext::fsm_state_pack<
    PRL_Tx_PHY_Layer_Reset_State,
    PRL_Tx_Wait_for_Message_Request_State,
    PRL_Tx_Layer_Reset_for_Transmit_State,
    PRL_Tx_Construct_Message_State,
    PRL_Tx_Wait_for_PHY_Response_State,
    PRL_Tx_Match_MessageID_State,
    PRL_Tx_Message_Sent_State,
    PRL_Tx_Check_RetryCounter_State,
    PRL_Tx_Transmission_Error_State,
    PRL_Tx_Discard_Message_State,
    PRL_Tx_Snk_Start_of_AMS_State,
    PRL_Tx_Snk_Pending_State
> prl_tx_state_list;

PRL_Tx::PRL_Tx(PRL& prl) : etl::fsm(0), prl{prl} {
    set_states(prl_tx_state_list.states, prl_tx_state_list.size);
}

void PRL_Tx::log_state() {
    PRL_LOGI("PRL_Tx state => {}", prl_tx_state_to_desc(get_state_id()));
}

etl_ext::fsm_state_pack<
    PRL_Rx_Wait_for_PHY_Message_State,
    PRL_Rx_Layer_Reset_for_Receive_State,
    PRL_Rx_Send_GoodCRC_State,
    PRL_Rx_Check_MessageID_State,
    PRL_Rx_Store_MessageID_State
> prl_rx_state_list;

PRL_Rx::PRL_Rx(PRL& prl) : etl::fsm(0), prl{prl} {
    set_states(prl_rx_state_list.states, prl_rx_state_list.size);
}

void PRL_Rx::log_state() {
    PRL_LOGI("PRL_Rx state => {}", prl_rx_state_to_desc(get_state_id()));
}

etl_ext::fsm_state_pack<
    PRL_HR_IDLE_State,
    PRL_HR_Reset_Layer_State,
    PRL_HR_Indicate_Hard_Reset_State,
    PRL_HR_Request_Hard_Reset_State,
    PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State,
    PRL_HR_PHY_Hard_Reset_Requested_State,
    PRL_HR_Wait_for_PE_Hard_Reset_Complete_State,
    PRL_HR_PE_Hard_Reset_Complete_State
> prl_hr_state_list;

PRL_HR::PRL_HR(PRL& prl) : etl::fsm(0), prl{prl} {
    set_states(prl_hr_state_list.states, prl_hr_state_list.size);
}

void PRL_HR::log_state() {
    PRL_LOGI("PRL_HR state => {}", prl_hr_state_to_desc(get_state_id()));
}


void PRL_EventListener::on_receive(const MsgSysUpdate& msg) {
    switch (prl.local_state) {
        case PRL::LOCAL_STATE::DISABLED:
            if (!prl.port.is_attached) { break; }

            __fallthrough;
        case PRL::LOCAL_STATE::INIT:
            prl.init();
            prl.local_state = PRL::LOCAL_STATE::WORKING;

            __fallthrough;
        case PRL::LOCAL_STATE::WORKING:
            if (!prl.port.is_attached) {
                prl.tcpc.req_rx_enable(false);
                prl.local_state = PRL::LOCAL_STATE::DISABLED;
                break;
            }

            prl.prl_hr.receive(msg);

            if (prl.prl_hr.get_state_id() != PRL_HR_IDLE) { break; }

            // In theory, if RTOS with slow reaction used, it's possible to get
            // both TX Complete and RX updates when transmission was requested

            if (prl.port.tcpc_tx_status.load() == TCPC_TRANSMIT_STATUS::SUCCEEDED)
            {
                // If TCPC send finished - ensure to react before discarding
                // by RX (if both events detected in the same time).
                //
                // - Skip TCPC fail here, because it can start retry.
                // - Skip TCPC discard here, to expose by RX
                //
                // May be software CRC handling needs more care. But for
                // hardware CRC this looks ok.
                prl.prl_tx.receive(msg);
            }

            prl.prl_rx.receive(msg);
            prl.prl_rch.receive(msg);
            // First TCH call needed when PE enquired message, to start
            // chunking / transfer.
            prl.prl_tch.receive(msg);
            prl.prl_tx.receive(msg);

            //
            // Repeat TCH/RCH calls for quick-consume previous changes
            //

            // After transfer complete - PE should be notified, call TCH again.
            prl.prl_tch.receive(msg);
            // Repeat RCH call to land
            // - re-routed TCH message
            // - prl_tx status update after chunk request
            prl.prl_rch.receive(msg);
            break;
    }
}

void PRL_EventListener::on_receive(const MsgToPrl_SoftResetFromPe&) {
    prl.local_state = PRL::LOCAL_STATE::INIT;
}

void PRL_EventListener::on_receive(const MsgToPrl_HardResetFromPe&) {
    prl.port.prl_hr_flags.set(PRL_HR_FLAG::HARD_RESET_FROM_PE);
}

void PRL_EventListener::on_receive(const MsgToPrl_PEHardResetDone&) {
    prl.port.prl_hr_flags.set(PRL_HR_FLAG::PE_HARD_RESET_COMPLETE);
}

void PRL_EventListener::on_receive(const MsgToPrl_TcpcHardReset&) {
    prl.port.prl_hr_flags.set(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER);
    prl.port.wakeup();
}

void PRL_EventListener::on_receive(const MsgToPrl_CtlMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    prl.port.tx_emsg.clear();
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
    prl.port.wakeup();
}

void PRL_EventListener::on_receive(const MsgToPrl_DataMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
    prl.port.wakeup();
}

void PRL_EventListener::on_receive(const MsgToPrl_ExtMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    hdr.extended = 1;
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
    prl.port.wakeup();
}

void PRL_EventListener::on_receive(const MsgToPrl_GetPrlStatus& msg) {
    msg.is_running_ref = (prl.local_state == PRL::LOCAL_STATE::WORKING);
    msg.is_busy_ref =
        (prl.prl_rch.get_state_id() != RCH_Wait_For_Message_From_Protocol_Layer) ||
        (prl.prl_tch.get_state_id() != TCH_Wait_For_Message_Request_From_Policy_Engine);
}

void PRL_EventListener::on_receive_unknown(__maybe_unused const etl::imessage& msg) {
    PRL_LOGE("PRL unknown message, id: {}", msg.get_message_id());
}

} // namespace pd
