#include "pd_conf.h"
#include "pe.h"
#include "prl.h"
#include "sink.h"
#include "tc.h"
#include "common_macros.h"

#include <etl/array.h>

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
    PRL_RCH_STATE_COUNT
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
    PRL_TCH_STATE_COUNT
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
    PRL_Tx_STATE_COUNT
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
    PRL_Rx_STATE_COUNT
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
    PRL_HR_STATE_COUNT
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
auto on_event_unknown(__maybe_unused const etl::imessage& event) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}

#define ON_TRANSIT_TO \
auto on_event(const MsgTransitTo& event) -> etl::fsm_state_id_t { \
    return event.state_id; \
}

#define ON_EVENT_NOTHING \
auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.1.2 Chunked Rx State Diagram

class RCH_Wait_For_Message_From_Protocol_Layer_State : public etl::fsm_state<PRL_RCH, RCH_Wait_For_Message_From_Protocol_Layer_State, RCH_Wait_For_Message_From_Protocol_Layer, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        rch.log_state();

        rch.prl.flags.clear(PRL_FLAG::ABORT);
        rch.prl.rx_emsg.data.clear();
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();

        if (rch.flags.test_and_clear(RCH_FLAG::RX_ENQUEUED)) {
            auto& prl = rch.prl;
            auto& chunk = prl.rx_chunk;

            // Copy header to output struct
            prl.rx_emsg.header = chunk.header;

            if (chunk.header.extended) {
                PD_EXT_HEADER ehdr{.raw_value = chunk.read16(0)};

                if (ehdr.chunked && rch.prl.chunking()) {
                    // Spec says clear vars below in RCH_Processing_Extended_Message
                    // on first chunk, but this place looks more obvious
                    rch.chunk_number_expected = 0;
                    rch.prl.tx_emsg.header = chunk.header;
                    return RCH_Processing_Extended_Message;
                }

                if (!ehdr.chunked && !rch.prl.chunking()) {
                    // Unchunked ext messages are not supported now
                    rch.error = PRL_ERROR::RCH_BAD_SEQUENCE;
                    return RCH_Report_Error;
                }

                rch.error = PRL_ERROR::RCH_BAD_SEQUENCE;
                return RCH_Report_Error;
            }

            // Non-extended message
            rch.copy_chunk_data();
            return RCH_Pass_Up_Message;
        }
        return No_State_Change;
    }
};

class RCH_Pass_Up_Message_State : public etl::fsm_state<PRL_RCH, RCH_Pass_Up_Message_State, RCH_Pass_Up_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        rch.log_state();

        rch.prl.sink.pe->on_message_received();
        return RCH_Wait_For_Message_From_Protocol_Layer;
    }
};

class RCH_Processing_Extended_Message_State : public etl::fsm_state<PRL_RCH, RCH_Processing_Extended_Message_State, RCH_Processing_Extended_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        rch.log_state();

        auto& chunk = rch.prl.rx_chunk;
        auto& msg = rch.prl.rx_emsg;

        PD_EXT_HEADER ehdr{.raw_value = chunk.read16(0)};

        if (rch.prl.flags.test_and_clear(PRL_FLAG::ABORT)) {
            return TCH_Wait_For_Transmision_Complete;
        }

        // Minimal data integrity check
        if ((ehdr.chunk_number != rch.chunk_number_expected) ||
            (ehdr.chunk_number >= MaxChunksPerMsg) ||
            (ehdr.data_size > MaxExtendedMsgLen))
        {
            rch.error = PRL_ERROR::RCH_BAD_SEQUENCE;
            return RCH_Report_Error;
        }

        // Copy as much as possible (without ext header),
        // until desired size reached.
        msg.data.insert(msg.data.end(), chunk.data.begin() + 2, chunk.data.end());
        rch.chunk_number_expected++;

        if (msg.data.size() >= ehdr.data_size) {
            msg.data.resize(ehdr.data_size);
            return RCH_Pass_Up_Message;
        }
        return RCH_Requesting_Chunk;
    }
};

class RCH_Requesting_Chunk_State : public etl::fsm_state<PRL_RCH, RCH_Requesting_Chunk_State, RCH_Requesting_Chunk, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        rch.log_state();

        auto& prl = rch.prl;
        // Block PE timeout timer for multichunk responses, it should not fail
        prl.sink.timers.stop(PD_TIMEOUT::tSenderResponse);

        auto& chunk = prl.tx_chunk;

        PD_HEADER hdr{.raw_value = 0};
        hdr.message_type = prl.rx_emsg.header.message_type;
        hdr.data_obj_count = 1;
        hdr.extended = 1;

        PD_EXT_HEADER ehdr{.raw_value = 0};
        ehdr.request_chunk = 1;
        ehdr.chunk_number = rch.chunk_number_expected;
        ehdr.chunked = 1;

        chunk.header = hdr;
        chunk.data.clear();
        chunk.append16(ehdr.raw_value);
        chunk.append16(0); // Placeholder, align to 32 bit (data object size)

        // Mark chunk for send
        prl.tx_enquire_chunk();
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& prl_tx = rch.prl.prl_tx;

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            return RCH_Waiting_Chunk;
        }

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            rch.error = PRL_ERROR::RCH_SEND_FAIL;
            return RCH_Report_Error;
        }

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_DISCARDED)) {
            // Discarded chunk request in chunked RX is abnormal fuckup.
            // Report it via error only, not via discard.
            PRL_LOG("RCH: Chunk request discarded");
            // Instead of error reporting here, go to next state to
            // handle new message properly.
            return RCH_Waiting_Chunk;
        }

        return No_State_Change;
    }
};

class RCH_Waiting_Chunk_State : public etl::fsm_state<PRL_RCH, RCH_Waiting_Chunk_State, RCH_Waiting_Chunk, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& rch = get_fsm_context();
        rch.log_state();

        rch.prl.sink.timers.start(PD_TIMEOUT::tChunkSenderResponse);
        rch.prl.sink.timers.start(PD_TIMEOUT::tSenderResponse);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();

        if (rch.flags.test(RCH_FLAG::RX_ENQUEUED)) {
            auto& chunk = rch.prl.rx_chunk;
            auto& hdr = chunk.header;

            // Spec requires to inform PE immediately about new message on
            // wrong sequence, prior returning to
            // RCH_Wait_For_Message_From_Protocol_Layer.
            // But we can safely land only unchunked messages this way.

            // NOTE: if unchunked ext msg eve supported, filter here too.
            if (hdr.extended == 0) {
                rch.error = PRL_ERROR::RCH_SEQUENCE_DISCARDED;
                return RCH_Report_Error;
            }

            rch.flags.clear(RCH_FLAG::RX_ENQUEUED);
            return RCH_Processing_Extended_Message;
        }

        if (rch.prl.sink.timers.is_expired(PD_TIMEOUT::tChunkSenderResponse)) {
            rch.error = PRL_ERROR::RCH_SEQUENCE_TIMEOUT;
            return RCH_Report_Error;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& rch = get_fsm_context();
        rch.prl.sink.timers.stop(PD_TIMEOUT::tChunkSenderResponse);
    }
};

class RCH_Report_Error_State : public etl::fsm_state<PRL_RCH, RCH_Report_Error_State, RCH_Report_Error, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& rch = get_fsm_context();
        auto& prl = rch.prl;

        if (prl.flags.test_and_clear(RCH_FLAG::RX_ENQUEUED)) {
            rch.copy_chunk_data();
            prl.sink.pe->on_message_received();
        }

        // Error report can cause sending data (soft reset, for example)
        // It's important to finish with initial state without pending chunks.
        // Otherwise, TCH will reject request.

        // Rise flag instead of direct call, to avoid possible recursion
        rch.flags.set(RCH_FLAG::RCH_ERROR_PENDING);

        return RCH_Wait_For_Message_From_Protocol_Layer;
    }
};


////////////////////////////////////////////////////////////////////////////////

class TCH_Wait_For_Message_Request_From_Policy_Engine_State : public etl::fsm_state<PRL_TCH, TCH_Wait_For_Message_Request_From_Policy_Engine_State, TCH_Wait_For_Message_Request_From_Policy_Engine, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        tch.log_state();

        tch.prl.flags.clear(PRL_FLAG::ABORT);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;

        if (tch.flags.test_and_clear(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
            return TCH_Message_Received;
        }

        if (tch.flags.test_and_clear(TCH_FLAG::MSG_ENQUEUED)) {
            if (prl.prl_rch.get_state_id() != RCH_Wait_For_Message_From_Protocol_Layer) {
                tch.prl.sink.pe->on_prl_report_discard();
                tch.error = PRL_ERROR::TCH_ENQUIRE_DISCARDED;
                return TCH_Report_Error;
            }

            if (prl.tx_emsg.header.extended && prl.chunking()) {
                return TCH_Prepare_To_Send_Chunked_Message;
            }
            return TCH_Pass_Down_Message;
        }
        return No_State_Change;
    }
};

class TCH_Pass_Down_Message_State : public etl::fsm_state<PRL_TCH, TCH_Pass_Down_Message_State, TCH_Pass_Down_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;
        tch.log_state();

        // NOTE: add unchunked ext msg data copy

        // Copy data to chunk & fill data object count
        prl.tx_chunk.header = prl.tx_emsg.header;
        prl.tx_chunk.data.assign(prl.tx_emsg.data.begin(), prl.tx_emsg.data.end());
        prl.tx_chunk.header.data_obj_count = (prl.tx_emsg.data.size() + 3) >> 2;

        prl.prl_tx.flags.set(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
        return TCH_Wait_For_Transmision_Complete;
    }
};

class TCH_Wait_For_Transmision_Complete_State : public etl::fsm_state<PRL_TCH, TCH_Wait_For_Transmision_Complete_State, TCH_Wait_For_Transmision_Complete, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& prl_tx = tch.prl.prl_tx;

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            return TCH_Message_Sent;
        }

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            tch.error = PRL_ERROR::TCH_SEND_FAIL;
            return TCH_Report_Error;
        }

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_DISCARDED)) {
            tch.prl.sink.pe->on_prl_report_discard();
            tch.error = PRL_ERROR::TCH_DISCARDED;
            return TCH_Report_Error;
        }

        return No_State_Change;
    }
};

class TCH_Message_Sent_State : public etl::fsm_state<PRL_TCH, TCH_Message_Sent_State, TCH_Message_Sent, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        tch.log_state();

        tch.prl.sink.pe->on_message_sent();
        if (tch.flags.test(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
            return TCH_Message_Received;
        }
        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }
};

class TCH_Prepare_To_Send_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Prepare_To_Send_Chunked_Message_State, TCH_Prepare_To_Send_Chunked_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        tch.log_state();

        tch.chunk_number_to_send = 0;
        return TCH_Construct_Chunked_Message;
    }
};

class TCH_Construct_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Construct_Chunked_Message_State, TCH_Construct_Chunked_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;
        tch.log_state();

        int tail_len = prl.tx_emsg.data.size() - tch.chunk_number_to_send * MaxExtendedMsgChunkLen;
        int chunk_data_len = etl::min(tail_len, MaxExtendedMsgChunkLen);

        PD_EXT_HEADER ehdr{.raw_value = 0};
        ehdr.data_size = prl.tx_emsg.data.size();
        ehdr.chunk_number = tch.chunk_number_to_send;
        ehdr.chunked = 1;

        prl.tx_chunk.data.clear();
        prl.tx_chunk.append16(ehdr.raw_value);
        auto offset = tch.chunk_number_to_send * MaxExtendedMsgChunkLen;
        prl.tx_chunk.data.insert(prl.tx_chunk.data.end(),
            prl.tx_emsg.data.begin() + offset,
            prl.tx_emsg.data.begin() + offset + chunk_data_len);

        prl.tx_chunk.header = prl.tx_emsg.header;
        prl.tx_chunk.header.data_obj_count = (prl.tx_chunk.data.size() + 3) >> 2;

        if (tch.prl.flags.test_and_clear(PRL_FLAG::ABORT)) {
            return TCH_Wait_For_Transmision_Complete;
        }

        prl.prl_tx.flags.set(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
        return No_State_Change;
    }
};

class TCH_Sending_Chunked_Message_State : public etl::fsm_state<PRL_TCH, TCH_Sending_Chunked_Message_State, TCH_Sending_Chunked_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;
        auto& prl_tx = prl.prl_tx;

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_DISCARDED)) {
            tch.prl.sink.pe->on_prl_report_discard();
            tch.error = PRL_ERROR::TCH_DISCARDED;
            return TCH_Report_Error;
        }

        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            tch.error = PRL_ERROR::TCH_SEND_FAIL;
            return TCH_Report_Error;
        }

        // Calculate max possible bytes been sent, if all chunks are of max size
        int max_bytes = (tch.chunk_number_to_send + 1) * MaxExtendedMsgChunkLen;

        // Reached msg size => last chunk sent
        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            if (max_bytes >= prl.tx_emsg.data.size()) {
                return TCH_Message_Sent;
            }

            if (tch.flags.test(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
                // If tx/rx events received at once, kick loop to ensure
                // next state is executed.
                tch.prl.sink.wakeup();
            }
            return TCH_Wait_Chunk_Request;
        }

        return No_State_Change;
    }
};

class TCH_Wait_Chunk_Request_State : public etl::fsm_state<PRL_TCH, TCH_Wait_Chunk_Request_State, TCH_Wait_Chunk_Request, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        tch.log_state();

        tch.chunk_number_to_send++;
        tch.prl.sink.timers.start(PD_TIMEOUT::tChunkSenderRequest);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;

        if (tch.flags.test(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
            if (prl.rx_emsg.header.extended) {
                PD_EXT_HEADER ehdr{.raw_value = prl.rx_chunk.read16(0)};

                if (ehdr.request_chunk) {
                    if (ehdr.chunk_number == tch.chunk_number_to_send) {
                        tch.flags.clear(TCH_FLAG::NEXT_CHUNK_REQUEST);
                        return TCH_Construct_Chunked_Message;
                    }

                    tch.flags.clear(TCH_FLAG::NEXT_CHUNK_REQUEST);
                    tch.error = PRL_ERROR::TCH_BAD_SEQUENCE;
                    return TCH_Report_Error;
                }
            }

            // Not expected message
            tch.prl.sink.pe->on_prl_report_discard();
            tch.error = PRL_ERROR::TCH_DISCARDED;
            return TCH_Message_Received;
        }

        if (tch.prl.sink.timers.is_expired(PD_TIMEOUT::tChunkSenderRequest)) {
            tch.error = PRL_ERROR::TCH_SEQUENCE_TIMEOUT;
            return TCH_Report_Error;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& tch = get_fsm_context();
        tch.prl.sink.timers.stop(PD_TIMEOUT::tChunkSenderRequest);
    }
};

class TCH_Message_Received_State : public etl::fsm_state<PRL_TCH, TCH_Message_Received_State, TCH_Message_Received, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        auto& prl = tch.prl;
        tch.log_state();

        // Forward msg to RCH
        if (tch.flags.test_and_clear(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
            prl.prl_rch.flags.set(RCH_FLAG::RX_ENQUEUED);
            prl.sink.wakeup();
        }

        if (tch.flags.test_and_clear(TCH_FLAG::MSG_ENQUEUED)) {
            prl.sink.pe->on_prl_report_discard();
        }

        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }
};

class TCH_Report_Error_State : public etl::fsm_state<PRL_TCH, TCH_Report_Error_State, TCH_Report_Error, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& tch = get_fsm_context();
        tch.log_state();

        // Rise flag instead of direct call, to avoid possible recursion
        tch.flags.set(TCH_FLAG::TCH_ERROR_PENDING);

        if (tch.flags.test(TCH_FLAG::NEXT_CHUNK_REQUEST)) {
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

class PRL_Tx_PHY_Layer_Reset_State : public etl::fsm_state<PRL_Tx, PRL_Tx_PHY_Layer_Reset_State, PRL_Tx_PHY_Layer_Reset, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        prl_tx.prl.tcpc.set_rx_enable(true);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        if (prl.tcpc.get_state().test(TCPC_CALL_FLAG::SET_RX_ENABLE)) {
            return No_State_Change;
        }

        // Kick to guarantee next state run
        prl.sink.wakeup();
        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Wait_for_Message_Request_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Wait_for_Message_Request_State, PRL_Tx_Wait_for_Message_Request, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        prl_tx.tcpc_tx_status = TCPC_TRANSMIT_STATUS::UNSET;
        prl_tx.retry_counter = 0;

        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();

        // For first AMS message need to wait SinkTxOK CC level
        if (!prl_tx.prl.sink.pe->is_ams_active()) {
            prl_tx.flags.clear(PRL_TX_FLAG::START_OF_AMS_DETECTED);
        } else {
            if (!prl_tx.flags.test(PRL_TX_FLAG::START_OF_AMS_DETECTED)) {
                prl_tx.flags.set(PRL_TX_FLAG::START_OF_AMS_DETECTED);
                return PRL_Tx_Snk_Start_of_AMS;
            }
        }

        // For non-AMS messages, or after first AMS message
        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            auto& tx_chunk = prl_tx.prl.tx_chunk;

            if (tx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
                return PRL_Tx_Layer_Reset_for_Transmit;
            }
            return PRL_Tx_Construct_Message;
        }

        return No_State_Change;
    }
};

class PRL_Tx_Layer_Reset_for_Transmit_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Layer_Reset_for_Transmit_State, PRL_Tx_Layer_Reset_for_Transmit, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Note, spec says to reset only `msg_id_counter` here, and reset
        // `msg_id_stored` via RX state change. But etl::fsm does not re-run
        // `on_enter` if we come from current state to itself.
        // So, reset both here.
        prl_tx.prl.reset_msg_counters();
        // This will not make sense, because we do not send GoodCRC in software,
        // and every input packet causes returns to initial state immediately.
        // But this is left for consistency with the spec.
        prl_tx.prl.prl_rx.receive(MsgTransitTo(PRL_Rx_Wait_for_PHY_Message));

        return PRL_Tx_Construct_Message;
    }
};

class PRL_Tx_Construct_Message_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Construct_Message_State, PRL_Tx_Construct_Message, MsgPdEvents, MsgTransitTo> {
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
class PRL_Tx_Wait_for_PHY_Response_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Wait_for_PHY_Response_State, PRL_Tx_Wait_for_PHY_Response, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Timer should be used ONLY when hardware confirmation not supported
        if (!prl_tx.prl.tcpc.get_hw_features().tx_goodcrc_receive) {
            prl_tx.prl.sink.timers.start(PD_TIMEOUT::tReceive);
        }
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();

        if (prl_tx.tcpc_tx_status == TCPC_TRANSMIT_STATUS::SUCCEEDED) {
            return PRL_Tx_Match_MessageID;
        }

        //
        // NOTE: do NOT check TCPC_TRANSMIT_STATUS::DISCARDED.
        // Rely on teleport to PRL_Tx_Discard_Message by prl_rx,
        // to have new msg + discard in sync.
        //

        if (prl_tx.tcpc_tx_status == TCPC_TRANSMIT_STATUS::FAILED) {
            return PRL_Tx_Check_RetryCounter;
        }

        // Actual only for software CRC processing
        if (prl_tx.prl.sink.timers.is_expired(PD_TIMEOUT::tReceive)) {
            return PRL_Tx_Check_RetryCounter;
        }

        return No_State_Change;
    }

    auto on_exit_state() -> void {
        auto& prl_tx = get_fsm_context();
        prl_tx.prl.sink.timers.stop(PD_TIMEOUT::tReceive);
    }
};

class PRL_Tx_Match_MessageID_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Match_MessageID_State, PRL_Tx_Match_MessageID, MsgPdEvents, MsgTransitTo> {
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

class PRL_Tx_Message_Sent_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Message_Sent_State, PRL_Tx_Message_Sent, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        prl_tx.msg_id_counter++;
        prl_tx.flags.set(PRL_TX_FLAG::TX_COMPLETED);

        // Ensure one more loop run, when RCH requests next chunk
        prl_tx.prl.sink.wakeup();

        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Check_RetryCounter_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Check_RetryCounter_State, PRL_Tx_Check_RetryCounter, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
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
            // Don't try retransmit if supported by hardware.
            return PRL_Tx_Transmission_Error;
        }

        prl_tx.retry_counter++;

        if (prl_tx.retry_counter > 2 /* nRetryCount */) {
            return PRL_Tx_Transmission_Error;
        }
        return PRL_Tx_Construct_Message;
    }
};

class PRL_Tx_Transmission_Error_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Transmission_Error_State, PRL_Tx_Transmission_Error, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        prl_tx.msg_id_counter++;
        // Don't report PE about error here, allow RCH/TCH to handle & care.
        prl_tx.flags.set(PRL_TX_FLAG::TX_ERROR);

        return PRL_Tx_Wait_for_Message_Request;
    }
};

class PRL_Tx_Discard_Message_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Discard_Message_State, PRL_Tx_Discard_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();

        // Do discard, if any TX chunk processing:
        // - input queued to send
        // - passed to driver, and sending in progress
        // - if driver signaled discard
        if (prl_tx.flags.test_and_clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED) ||
            prl_tx.tcpc_tx_status == TCPC_TRANSMIT_STATUS::WAITING ||
            prl_tx.tcpc_tx_status == TCPC_TRANSMIT_STATUS::DISCARDED)
        {
            prl_tx.msg_id_counter++;
            //prl_tx.prl.sink.pe->on_prl_report_discard();
            prl_tx.flags.set(PRL_TX_FLAG::TX_DISCARDED);
        }
        return PRL_Tx_PHY_Layer_Reset;
    }
};

class PRL_Tx_Snk_Start_of_AMS_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Snk_Start_of_AMS_State, PRL_Tx_Snk_Start_of_AMS, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Reuse existing event to switch state, if condition satisfied
        if (prl_tx.flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();

        if (prl_tx.flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }
};

class PRL_Tx_Snk_Pending_State : public etl::fsm_state<PRL_Tx, PRL_Tx_Snk_Pending_State, PRL_Tx_Snk_Pending, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        prl_tx.log_state();

        // Initiate CC reading. First call is required to ensure
        // cache is in sync. Subsequent updates can be interrupt-based.
        prl_tx.prl.tcpc.req_cc_active();
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_tx = get_fsm_context();
        auto& tcpc = prl_tx.prl.tcpc;

        // Wait until CC fetch completes
        if (tcpc.get_state().test(TCPC_CALL_FLAG::REQ_CC_ACTIVE)) {
            return No_State_Change;
        }

        // Soft reset passed without delay
        if (prl_tx.prl.tx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
            prl_tx.flags.clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
            return PRL_Tx_Layer_Reset_for_Transmit;
        }

        // Wait SinkTxOK before sending first AMS message
        if (tcpc.get_cc(TCPC_CC::ACTIVE) == TCPC_CC_LEVEL::SinkTxOK) {
            prl_tx.flags.clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
            return PRL_Tx_Construct_Message;
        }

        // Note, all PD controllers support interrupts on CC level change.
        // This manual request is left for sure, and not expected to be used.
        // Note, manual polling can cause unexpected CPU / I2C load.
        if (!tcpc.get_hw_features().cc_update_event) {
            tcpc.req_cc_active();
        }
        return No_State_Change;
    }
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.3 Protocol Layer Message Reception

class PRL_Rx_Wait_for_PHY_Message_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Wait_for_PHY_Message_State, PRL_Rx_Wait_for_PHY_Message, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        if (!prl.tcpc.has_rx_data()) { return No_State_Change; };

        if (prl.prl_rch.flags.test(RCH_FLAG::RX_ENQUEUED)) {
            // In theory, we can have pending packet in RCH, re-routed by
            // discard. Postpone processing of new one to next cycle, to allow
            // RCH to finish.
            //
            // This is not expected to happen, because we do multiple RCH/TCH
            // calls.
            prl.sink.wakeup();
            return No_State_Change;
        }

        prl.tcpc.fetch_rx_data(prl.rx_chunk);

        if (prl.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
            return PRL_Rx_Layer_Reset_for_Receive;
        }

        return PRL_Rx_Send_GoodCRC;
    }
};

class PRL_Rx_Layer_Reset_for_Receive_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Layer_Reset_for_Receive_State, PRL_Rx_Layer_Reset_for_Receive, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        prl_rx.log_state();

        auto& prl = prl_rx.prl;

        // Similar to init, but skip RX and (?) revision clear.
        prl.prl_tx.reset();
        prl.prl_tx.flags.clear_all();
        prl.prl_tx.start();

        prl.prl_rch.reset();
        prl.prl_rch.flags.clear_all();
        prl.prl_rch.start();

        prl.prl_tch.reset();
        prl.prl_tch.flags.clear_all();
        prl.prl_tch.start();

        prl.reset_msg_counters();

        prl.sink.pe->on_prl_soft_reset_from_partner();
        return PRL_Rx_Send_GoodCRC;
    }
};

class PRL_Rx_Send_GoodCRC_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Send_GoodCRC_State, PRL_Rx_Send_GoodCRC, MsgPdEvents, MsgTransitTo> {
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

class PRL_Rx_Check_MessageID_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Check_MessageID_State, PRL_Rx_Check_MessageID, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        prl_rx.log_state();

        if (prl_rx.msg_id_stored == prl_rx.prl.rx_chunk.header.message_id) {
            // Ignore duplicated
            return PRL_Rx_Wait_for_PHY_Message;
        }
        return PRL_Rx_Store_MessageID;
    }
};

class PRL_Rx_Store_MessageID_State : public etl::fsm_state<PRL_Rx, PRL_Rx_Store_MessageID_State, PRL_Rx_Store_MessageID, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& prl_rx = get_fsm_context();
        prl_rx.log_state();

        auto& prl = prl_rx.prl;

        prl_rx.msg_id_stored = prl.rx_chunk.header.message_id;

        // Rev 3.2 says ping is deprecated => Ignore it completely
        // (it should not discard, affect chunking and so on).
        if (prl.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Ping)) { return PRL_Rx_Wait_for_PHY_Message; }

        prl.prl_tx.receive(MsgTransitTo(PRL_Tx_Discard_Message));

        // [rev3.2] 6.12.2.1.4 Chunked Message Router State Diagram
        //
        // Route message to RCH/TCH. Since RTR has no stored states, it is
        // more simple to embed it's logic here.

        // Spec describes chunking as "Not in initial waiting state". But
        // PE send requests are not executed immediately, those just rise flag.
        // So, having that flag set means "not waiting" too.
        if (prl.prl_tch.flags.test(TCH_FLAG::MSG_ENQUEUED ||
            prl.prl_tch.get_state_id() != TCH_Wait_For_Message_Request_From_Policy_Engine))
        {
            // TCH does chunking => route to it
            prl.prl_tch.flags.set(TCH_FLAG::NEXT_CHUNK_REQUEST);
        } else {
            // No TCH chunking => route to RCH
            prl.prl_rch.flags.set(RCH_FLAG::RX_ENQUEUED);
        }

        // Return back to waiting.
        return PRL_Rx_Wait_for_PHY_Message;

    }
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.4 Hard Reset operation

class PRL_HR_IDLE_State : public etl::fsm_state<PRL_HR, PRL_HR_IDLE_State, PRL_HR_IDLE, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl_hr = get_fsm_context().prl.prl_hr;
        if (prl_hr.flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER) ||
            prl_hr.flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PE))
        {
            return PRL_HR_Reset_Layer;
        }
        return No_State_Change;
    }
};

class PRL_HR_Reset_Layer_State : public etl::fsm_state<PRL_HR, PRL_HR_Reset_Layer_State, PRL_HR_Reset_Layer, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        // First, we don't "reset" anything in this state, because PRL_TX fsm
        // reset will cause enabling RX back. Until PRL_HR returns to initial
        // state, events are not precessed, and that looks safe.

        // Clear flags for sure, but that seems not necessary.
        hr.prl.flags.clear_all();
        hr.prl.prl_tx.flags.clear_all();
        hr.prl.prl_rch.flags.clear_all();
        hr.prl.prl_tch.flags.clear_all();

        // Start with RX path disable (and FIFO clear).
        hr.prl.tcpc.set_rx_enable(false);

        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        // Wait for TCPC operation complete
        if (prl.tcpc.get_state().test(TCPC_CALL_FLAG::SET_RX_ENABLE)) {
            return No_State_Change;
        }

        // Route state, depending on hard reset type requested
        if (prl.prl_hr.flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER)) {
            return PRL_HR_Indicate_Hard_Reset;
        }
        return PRL_HR_Request_Hard_Reset;
    }
};

class PRL_HR_Indicate_Hard_Reset_State : public etl::fsm_state<PRL_HR, PRL_HR_Indicate_Hard_Reset_State, PRL_HR_Indicate_Hard_Reset, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.sink.pe->on_prl_hard_reset_from_partner();
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }
};

class PRL_HR_Request_Hard_Reset_State : public etl::fsm_state<PRL_HR, PRL_HR_Request_Hard_Reset_State, PRL_HR_Request_Hard_Reset, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.tcpc.hard_reset();
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;

        // Wait for TCPC call to complete
        if (prl.tcpc.get_state().test(TCPC_CALL_FLAG::HARD_RESET)) {
            return No_State_Change;
        }
        return PRL_HR_Wait_for_PHY_Hard_Reset_Complete;
    }
};

class PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State, PRL_HR_Wait_for_PHY_Hard_Reset_Complete, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.sink.timers.start(PD_TIMEOUT::tHardResetComplete);
        return No_State_Change;
    }

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& prl = get_fsm_context().prl;
        if (prl.sink.timers.is_expired(PD_TIMEOUT::tHardResetComplete)) {
            return PRL_HR_PHY_Hard_Reset_Requested;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& prl = get_fsm_context().prl;
        prl.sink.timers.stop(PD_TIMEOUT::tHardResetComplete);
    }
};

class PRL_HR_PHY_Hard_Reset_Requested_State : public etl::fsm_state<PRL_HR, PRL_HR_PHY_Hard_Reset_Requested_State, PRL_HR_PHY_Hard_Reset_Requested, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.sink.pe->on_prl_hard_reset_sent();
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }
};

class PRL_HR_Wait_for_PE_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_Wait_for_PE_Hard_Reset_Complete_State, PRL_HR_Wait_for_PE_Hard_Reset_Complete, MsgPdEvents, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT;

    auto on_event(__maybe_unused const MsgPdEvents& event) -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();

        if (hr.flags.test_and_clear(PRL_HR_FLAG::PE_HARD_RESET_COMPLETE)) {
            return PRL_HR_PE_Hard_Reset_Complete;
        }
        return No_State_Change;
    }
};

class PRL_HR_PE_Hard_Reset_Complete_State : public etl::fsm_state<PRL_HR, PRL_HR_PE_Hard_Reset_Complete_State, PRL_HR_PE_Hard_Reset_Complete, MsgPdEvents, MsgTransitTo> {
public:
    ON_TRANSIT_TO; ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& hr = get_fsm_context();
        hr.log_state();

        hr.prl.init(true);
        hr.flags.clear_all();
        return PRL_HR_IDLE;
    }
};


////////////////////////////////////////////////////////////////////////////////


PRL::PRL(Sink& sink, IDriver& tcpc)
    : sink{sink},
    tcpc{tcpc},
    prl_tx{*this},
    prl_rx{*this},
    prl_hr{*this},
    prl_rch{*this},
    prl_tch{*this}
{
    tcpc.set_msg_router(tcpc_event_handler);
}

void PRL::init(bool from_hr_fsm) {
    PE_LOG("PRL init");

    // If init called from PRL_HR, don't intrude HR fsm
    if (!from_hr_fsm) {
        prl_hr.reset();
        prl_hr.flags.clear_all();
        prl_hr.start();
    }

    prl_tx.reset();
    prl_tx.flags.clear_all();
    prl_tx.start();

    prl_rx.reset();
    prl_rx.start();

    prl_rch.reset();
    prl_rch.flags.clear_all();
    prl_rch.start();

    prl_tch.reset();
    prl_tch.flags.clear_all();
    prl_tch.start();

    reset_msg_counters();
    reset_revision();

    sink.timers.stop_range(PD_TIMERS_RANGE::PRL);

    // Ensure loop repeat to continue PE States, which wait for PRL run.
    sink.wakeup();
}

void PRL::dispatch(const MsgPdEvents& events, const bool pd_enabled) {
    switch (local_state) {
        case LS_DISABLED:
            if (!pd_enabled) { break; }

            __fallthrough;
        case LS_INIT:
            init();
            local_state = LS_WORKING;

            __fallthrough;
        case LS_WORKING:
            if (!pd_enabled) {
                tcpc.set_rx_enable(false);
                local_state = LS_DISABLED;
                break;
            }

            prl_hr.receive(events);

            if (prl_hr.get_state_id() != PRL_HR_IDLE) { break; }

            // In theory, if RTOS with slow reaction used, it's possible to get
            // both TX Complete and RX updates when transmission was requested

            if (prl_tx.tcpc_tx_status == TCPC_TRANSMIT_STATUS::SUCCEEDED)
            {
                // If TCPC send finished - ensure to react before discarding
                // by RX (if both events detected in the same time).
                //
                // - Skip TCPC fail here, because it can start retry.
                // - Skip TCPC discard here, to expose by RX
                //
                // May be software CRC handling needs more care. But for
                // hardware CRC this looks ok.
                prl_tx.receive(events);
            }

            prl_rx.receive(events);
            prl_rch.receive(events);
            report_pending_error();
            // First TCH call needed when PE enquired message, to start
            // chunking / transfer.
            prl_tch.receive(events);
            report_pending_error();
            prl_tx.receive(events);

            //
            // Repeat TCH/RCH calls for quick-consume previsous changes
            //

            // After transfer complete - PE should be notified, call TCH again.
            prl_tch.receive(events);
            report_pending_error();
            // Repeat RCH call to land re-routed TCH message
            prl_rch.receive(events);
            report_pending_error();
            break;
    }
}

void PRL::on_soft_reset_from_pe() {
    local_state = LS_INIT;
}

void PRL::on_hard_reset_from_pe() {
    prl_hr.flags.set(PRL_HR_FLAG::HARD_RESET_FROM_PE);
}
void PRL::on_pe_hard_reset_done() {
    prl_hr.flags.set(PRL_HR_FLAG::PE_HARD_RESET_COMPLETE);
}

void PRL::send_ctrl_msg(PD_CTRL_MSGT::Type msgt) {
    PD_HEADER hdr{};
    hdr.message_type = msgt;
    tx_emsg.header = hdr;
    tx_emsg.data.clear();

    prl_tch.flags.set(TCH_FLAG::MSG_ENQUEUED);
    sink.wakeup();
}

void PRL::send_data_msg(PD_DATA_MSGT::Type msgt) {
    PD_HEADER hdr{};
    hdr.message_type = msgt;
    tx_emsg.header = hdr;

    prl_tch.flags.set(TCH_FLAG::MSG_ENQUEUED);
    sink.wakeup();
}

void PRL::send_ext_msg(PD_EXT_MSGT::Type msgt) {
    PD_HEADER hdr{};
    hdr.message_type = msgt;
    hdr.extended = 1;
    tx_emsg.header = hdr;

    prl_tch.flags.set(TCH_FLAG::MSG_ENQUEUED);
    sink.wakeup();
}

void PRL::tcpc_enquire_msg() {
    tx_chunk.header.message_id = prl_tx.msg_id_counter;
    tx_chunk.header.spec_revision = revision;

    // Clear PRL_TX "output" flags
    prl_tx.flags.clear(PRL_TX_FLAG::TX_COMPLETED);
    prl_tx.flags.clear(PRL_TX_FLAG::TX_ERROR);

    // Rearm TCPC TX status.
    prl_tx.tcpc_tx_status = TCPC_TRANSMIT_STATUS::WAITING;

    tcpc.transmit(tx_chunk);
}

void PRL::tx_enquire_chunk() {
    prl_tx.flags.set(PRL_TX_FLAG::TX_COMPLETED);
    prl_tx.flags.set(PRL_TX_FLAG::TX_ERROR);
    prl_tx.flags.set(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);
    sink.wakeup();
}

void PRL::report_pending_error() {
    if (prl_rch.flags.test_and_clear(RCH_FLAG::RCH_ERROR_PENDING)) {
        sink.pe->on_prl_report_error(prl_rch.error);
    }
    if (prl_tch.flags.test_and_clear(TCH_FLAG::TCH_ERROR_PENDING)) {
        sink.pe->on_prl_report_error(prl_tch.error);
    }
}

////////////////////////////////////////////////////////////////////////////////
// FSMs configs

PRL_RCH::PRL_RCH(PRL& prl) : etl::fsm(0), prl{prl} {
    static etl::array<etl::ifsm_state*, PRL_RCH_STATE_COUNT> state_list = {{
        new RCH_Wait_For_Message_From_Protocol_Layer_State(),
        new RCH_Pass_Up_Message_State(),
        new RCH_Processing_Extended_Message_State(),
        new RCH_Requesting_Chunk_State(),
        new RCH_Waiting_Chunk_State(),
        new RCH_Report_Error_State()
    }};
    set_states(state_list.data(), PRL_RCH_STATE_COUNT);
};

void PRL_RCH::log_state() {
    PRL_LOG("PRL_RCH state => %s", prl_rch_state_to_desc(get_state_id()));
}

void PRL_RCH::copy_chunk_data() {
    auto& msg = prl.rx_emsg;
    auto& chunk = prl.rx_chunk;

    // Currently we cove only non-extended messages.

    // Adjust buffer size
    chunk.data.resize(chunk.header.data_obj_count * 4);

    msg.header = chunk.header;
    msg.data.assign(chunk.data.begin(), chunk.data.end());
}

PRL_TCH::PRL_TCH(PRL& prl) : etl::fsm(0), prl{prl} {
    static etl::array<etl::ifsm_state*, PRL_TCH_STATE_COUNT> state_list = {
        new TCH_Wait_For_Message_Request_From_Policy_Engine_State(),
        new TCH_Pass_Down_Message_State(),
        new TCH_Wait_For_Transmision_Complete_State(),
        new TCH_Message_Sent_State(),
        new TCH_Prepare_To_Send_Chunked_Message_State(),
        new TCH_Construct_Chunked_Message_State(),
        new TCH_Sending_Chunked_Message_State(),
        new TCH_Wait_Chunk_Request_State(),
        new TCH_Message_Received_State(),
        new TCH_Report_Error_State()
    };
    set_states(state_list.data(), PRL_TCH_STATE_COUNT);
}

void PRL_TCH::log_state() {
    PRL_LOG("PRL_TCH state => %s", prl_tch_state_to_desc(get_state_id()));
}

PRL_Tx::PRL_Tx(PRL& prl) : etl::fsm(0), prl{prl} {
    static etl::array<etl::ifsm_state*, PRL_Tx_STATE_COUNT> state_list = {
        new PRL_Tx_PHY_Layer_Reset_State(),
        new PRL_Tx_Wait_for_Message_Request_State(),
        new PRL_Tx_Layer_Reset_for_Transmit_State(),
        new PRL_Tx_Construct_Message_State(),
        new PRL_Tx_Wait_for_PHY_Response_State(),
        new PRL_Tx_Match_MessageID_State(),
        new PRL_Tx_Message_Sent_State(),
        new PRL_Tx_Check_RetryCounter_State(),
        new PRL_Tx_Transmission_Error_State(),
        new PRL_Tx_Discard_Message_State(),
        new PRL_Tx_Snk_Start_of_AMS_State(),
        new PRL_Tx_Snk_Pending_State()
    };
    set_states(state_list.data(), PRL_Tx_STATE_COUNT);
}

void PRL_Tx::log_state() {
    PRL_LOG("PRL_Tx state => %s", prl_tx_state_to_desc(get_state_id()));
}

PRL_Rx::PRL_Rx(PRL& prl) : etl::fsm(0), prl{prl} {
    static etl::array<etl::ifsm_state*, PRL_Rx_STATE_COUNT> state_list = {
        new PRL_Rx_Wait_for_PHY_Message_State(),
        new PRL_Rx_Layer_Reset_for_Receive_State(),
        new PRL_Rx_Send_GoodCRC_State(),
        new PRL_Rx_Check_MessageID_State(),
        new PRL_Rx_Store_MessageID_State()
    };
    set_states(state_list.data(), PRL_Rx_STATE_COUNT);
}

void PRL_Rx::log_state() {
    PRL_LOG("PRL_Rx state => %s", prl_rx_state_to_desc(get_state_id()));
}

PRL_HR::PRL_HR(PRL& prl) : etl::fsm(0), prl{prl} {
    static etl::array<etl::ifsm_state*, PRL_HR_STATE_COUNT> state_list = {
        new PRL_HR_IDLE_State(),
        new PRL_HR_Reset_Layer_State(),
        new PRL_HR_Indicate_Hard_Reset_State(),
        new PRL_HR_Request_Hard_Reset_State(),
        new PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State(),
        new PRL_HR_PHY_Hard_Reset_Requested_State(),
        new PRL_HR_Wait_for_PE_Hard_Reset_Complete_State(),
        new PRL_HR_PE_Hard_Reset_Complete_State()
    };
    set_states(state_list.data(), PRL_HR_STATE_COUNT);
}

void PRL_HR::log_state() {
    PRL_LOG("PRL_HR state => %s", prl_hr_state_to_desc(get_state_id()));
}

TcpcEventHandler::TcpcEventHandler(PRL& prl) : prl{prl} {};

void TcpcEventHandler::on_receive(__maybe_unused const MsgTcpcHardReset& msg) {
    prl.prl_hr.flags.set(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER);
    prl.sink.wakeup();
}
void TcpcEventHandler::on_receive(__maybe_unused const MsgTcpcWakeup& msg) {
    prl.sink.wakeup();
}
void TcpcEventHandler::on_receive(__maybe_unused const MsgTcpcTransmitStatus& msg) {
    prl.prl_tx.tcpc_tx_status = msg.status;
    prl.sink.wakeup();
}
void TcpcEventHandler::on_receive(__maybe_unused const MsgTimerEvent& msg) {
    prl.sink.set_event(PD_EVENT::TIMER);
}
void TcpcEventHandler::on_receive_unknown(__maybe_unused const etl::imessage& msg) {
    PRL_LOG("Unknown TCPC event");
}

} // namespace pd
