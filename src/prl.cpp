#include <etl/array.h>

#include "common_macros.h"
#include "idriver.h"
#include "pd_log.h"
#include "port.h"
#include "prl.h"
#include "utils/etl_state_pack.h"

namespace pd {

using afsm::state_id_t;

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
    // NOTE: rev3.2 spec has obvious typo, naming it as
    // TCH_Wait_For_Transmision_Complete (with single 's')
    TCH_Wait_For_Transmission_Complete,
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
            case TCH_Wait_For_Transmission_Complete: return "TCH_Wait_For_Transmission_Complete";
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


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.1.2 Chunked Rx State Diagram

class RCH_Wait_For_Message_From_Protocol_Layer_State : public afsm::state<PRL_RCH, RCH_Wait_For_Message_From_Protocol_Layer_State, RCH_Wait_For_Message_From_Protocol_Layer> {
public:
    // Spec requires to clear extended message buffer (rx_emsg) on enter,
    // but we do that on first chunk instead. Because buffer is shared with PE.
    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
        rch.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_RCH& rch) -> state_id_t {
        auto& port = rch.prl.port;

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

    static void on_exit_state(PRL_RCH&) {}
};

class RCH_Pass_Up_Message_State : public afsm::state<PRL_RCH, RCH_Pass_Up_Message_State, RCH_Pass_Up_Message> {
public:
    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
        rch.log_state();

        rch.prl.report_pe(MsgToPe_PrlMessageReceived{});
        return RCH_Wait_For_Message_From_Protocol_Layer;
    }

    static auto on_run_state(PRL_RCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_RCH&) {}
};

class RCH_Processing_Extended_Message_State : public afsm::state<PRL_RCH, RCH_Processing_Extended_Message_State, RCH_Processing_Extended_Message> {
public:
    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
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

    static auto on_run_state(PRL_RCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_RCH&) {}
};

class RCH_Requesting_Chunk_State : public afsm::state<PRL_RCH, RCH_Requesting_Chunk_State, RCH_Requesting_Chunk> {
public:


    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
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

    static auto on_run_state(PRL_RCH& rch) -> state_id_t {
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

    static void on_exit_state(PRL_RCH&) {}
};

class RCH_Waiting_Chunk_State : public afsm::state<PRL_RCH, RCH_Waiting_Chunk_State, RCH_Waiting_Chunk> {
public:


    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
        auto& port = rch.prl.port;
        rch.log_state();

        port.timers.start(PD_TIMEOUT::tChunkSenderResponse);
        port.timers.start(PD_TIMEOUT::tSenderResponse);
        return No_State_Change;
    }

    static auto on_run_state(PRL_RCH& rch) -> state_id_t {
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

    static void on_exit_state(PRL_RCH& rch) {
        rch.prl.port.timers.stop(PD_TIMEOUT::tChunkSenderResponse);
    }
};

class RCH_Report_Error_State : public afsm::state<PRL_RCH, RCH_Report_Error_State, RCH_Report_Error> {
public:
    static auto on_enter_state(PRL_RCH& rch) -> state_id_t {
        auto& port = rch.prl.port;

        if (port.prl_rch_flags.test_and_clear(RCH_FLAG::RX_ENQUEUED)) {
            port.rx_emsg = port.rx_chunk;
            port.rx_emsg.resize_by_data_obj_count();
            rch.prl.report_pe(MsgToPe_PrlMessageReceived{});
        }

        rch.prl.report_pe(MsgToPe_PrlReportError{port.rch_error});
        return RCH_Wait_For_Message_From_Protocol_Layer;
    }

    static auto on_run_state(PRL_RCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_RCH&) {}
};


////////////////////////////////////////////////////////////////////////////////

class TCH_Wait_For_Message_Request_From_Policy_Engine_State : public afsm::state<PRL_TCH, TCH_Wait_For_Message_Request_From_Policy_Engine_State, TCH_Wait_For_Message_Request_From_Policy_Engine> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        tch.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_TCH& tch) -> state_id_t {
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
                tch.prl.report_pe(MsgToPe_PrlReportDiscard{});
                return No_State_Change;
            }

            if (port.tx_emsg.header.extended) {
                return TCH_Prepare_To_Send_Chunked_Message;
            }
            return TCH_Pass_Down_Message;
        }
        return No_State_Change;
    }

    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Pass_Down_Message_State : public afsm::state<PRL_TCH, TCH_Pass_Down_Message_State, TCH_Pass_Down_Message> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        // Copy data to chunk & fill data object count
        port.tx_chunk = port.tx_emsg;
        port.tx_chunk.header.data_obj_count = port.tx_emsg.size_to_pdo_count();

        tch.prl.prl_tx_enquire_chunk();
        return TCH_Wait_For_Transmission_Complete;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Wait_For_Transmission_Complete_State : public afsm::state<PRL_TCH, TCH_Wait_For_Transmission_Complete_State, TCH_Wait_For_Transmission_Complete> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        tch.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_TCH& tch) -> state_id_t {
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

        // First, care about case when driver status is TCPC_TRANSMIT_STATUS::SUCCESS.
        // That means driver did transfer, but PRL_TX has not been called yet.
        // This is possible, because TX and RX events can arrive in the same
        // time. In this case just wait until PRL_TX called.
        if (port.tcpc_tx_status.load() == TCPC_TRANSMIT_STATUS::SUCCEEDED) {
            tch.prl.request_wakeup(); // Probably not needed, but just in case.
            return No_State_Change;
        }

        // At this point, if not finished TX but RX exists => discard happened
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            // At this point, discard already reported by PRL_TX
            return TCH_Message_Received;
        }

        return No_State_Change;
    }

    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Message_Sent_State : public afsm::state<PRL_TCH, TCH_Message_Sent_State, TCH_Message_Sent> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        tch.prl.report_pe(MsgToPe_PrlMessageSent{});

        // [rev3.2] 6.12.2.1.3 Chunked Tx State Diagram
        // Any Message Received and not in state TCH_Wait_Chunk_Request
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }
        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Prepare_To_Send_Chunked_Message_State : public afsm::state<PRL_TCH, TCH_Prepare_To_Send_Chunked_Message_State, TCH_Prepare_To_Send_Chunked_Message> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        port.tch_chunk_number_to_send = 0;
        return TCH_Construct_Chunked_Message;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Construct_Chunked_Message_State : public afsm::state<PRL_TCH, TCH_Construct_Chunked_Message_State, TCH_Construct_Chunked_Message> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
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
        // single data object size is 4 bytes
        port.tx_chunk.header.data_obj_count = port.tx_chunk.size_to_pdo_count();

        tch.prl.prl_tx_enquire_chunk();
        return TCH_Sending_Chunked_Message;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Sending_Chunked_Message_State : public afsm::state<PRL_TCH, TCH_Sending_Chunked_Message_State, TCH_Sending_Chunked_Message> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        tch.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_ERROR)) {
            port.tch_error = PRL_ERROR::TCH_SEND_FAIL;
            return TCH_Report_Error;
        }

        // The same approach as in TCH_Wait_For_Transmission_Complete_State
        // If transfer completed, but PRL_TX not yet executed before
        // TCH called - just allow it to happen.
        if (port.tcpc_tx_status.load() == TCPC_TRANSMIT_STATUS::SUCCEEDED &&
            !port.prl_tx_flags.test(PRL_TX_FLAG::TX_COMPLETED))
        {
            tch.prl.request_wakeup(); // Probably not needed, but just in case.
            return No_State_Change;
        }

        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_COMPLETED)) {
            // Calculate max possible bytes sent, if all chunks are of max size
            uint32_t max_bytes = (port.tch_chunk_number_to_send + 1) * MaxExtendedMsgChunkLen;

            // Reached msg size => last chunk sent. Land situation without error,
            // even if new message received.
            if (max_bytes >= port.tx_emsg.data_size()) {
                return TCH_Message_Sent;
            }

            // Not last chunk and probably can have incoming message
            return TCH_Wait_Chunk_Request;
        }

        // Not completed, but has incoming msg instead => discard happened.
        // on PRL_TX layer (most probable) OR at chunking layer (partner stopped
        // requesting sequence). For second case - report discard.
        // Duplicated discard reporting is not a problem (those are merged)
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            tch.prl.report_pe(MsgToPe_PrlReportDiscard{});
            return TCH_Message_Received;
        }


        return No_State_Change;
    }

    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Wait_Chunk_Request_State : public afsm::state<PRL_TCH, TCH_Wait_Chunk_Request_State, TCH_Wait_Chunk_Request> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        port.tch_chunk_number_to_send++;
        port.timers.start(PD_TIMEOUT::tChunkSenderRequest);

        // In edge case we could come here with RX already enqueued.
        // Then force loop wakeup() to ensure we continue processing.
        if (port.prl_tch_flags.test(TCH_FLAG::CHUNK_FROM_RX)) {
            tch.prl.request_wakeup();
        }
        return No_State_Change;
    }

    static auto on_run_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;

        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            if (port.rx_chunk.header.extended) {
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
            // chunks. Let's add discard for sure.
            tch.prl.report_pe(MsgToPe_PrlReportDiscard{});
            return TCH_Message_Received;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tChunkSenderRequest)) {
            port.tch_error = PRL_ERROR::TCH_SEQUENCE_TIMEOUT;
            return TCH_Report_Error;
        }

        return No_State_Change;
    }

    static void on_exit_state(PRL_TCH& tch) {
        tch.prl.port.timers.stop(PD_TIMEOUT::tChunkSenderRequest);
    }
};

class TCH_Message_Received_State : public afsm::state<PRL_TCH, TCH_Message_Received_State, TCH_Message_Received> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        // Forward PRL_RX message to RCH
        port.prl_rch_flags.set(RCH_FLAG::RX_ENQUEUED);
        tch.prl.request_wakeup();

        // Drop incoming TCH request from PE if any
        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::MSG_FROM_PE_ENQUEUED)) {
            tch.prl.report_pe(MsgToPe_PrlReportDiscard{});
        }

        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};

class TCH_Report_Error_State : public afsm::state<PRL_TCH, TCH_Report_Error_State, TCH_Report_Error> {
public:
    static auto on_enter_state(PRL_TCH& tch) -> state_id_t {
        auto& port = tch.prl.port;
        tch.log_state();

        tch.prl.report_pe(MsgToPe_PrlReportError{port.tch_error});

        if (port.prl_tch_flags.test_and_clear(TCH_FLAG::CHUNK_FROM_RX)) {
            return TCH_Message_Received;
        }
        return TCH_Wait_For_Message_Request_From_Policy_Engine;
    }

    static auto on_run_state(PRL_TCH&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_TCH&) {}
};


////////////////////////////////////////////////////////////////////////////////
// This is low level layer for packet rx/tx.
//
// - Only discards are reported to PE from here
// - Success/errors are forwarded to RCH/TCH via flags.
// - Some room is reserved for CRC processing, to stay close to spec. But
//   currently only branches for hardware-supported GoodCRC are actual.
//   This should be revisited and cleaned if software CRC support is not actual.

class PRL_Tx_PHY_Layer_Reset_State : public afsm::state<PRL_Tx, PRL_Tx_PHY_Layer_Reset_State, PRL_Tx_PHY_Layer_Reset> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        prl_tx.log_state();

        // Technically, we should call set_rx_enable(true). But since call is
        // async - postpone it for next state, to have vars init coordinated.
        return PRL_Tx_Wait_for_Message_Request;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Wait_for_Message_Request_State : public afsm::state<PRL_Tx, PRL_Tx_Wait_for_Message_Request_State, PRL_Tx_Wait_for_Message_Request> {
public:


    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
        port.tx_retry_counter = 0;

        if (prl_tx.get_previous_state_id() == PRL_Tx_PHY_Layer_Reset) {
            // This also resets fusb302 FIFO
            PRL_LOGD("Requesting RX enable");
            prl_tx.prl.tcpc.req_rx_enable(true);
        }

        return No_State_Change;
    }

    static auto on_run_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;

        if (!prl_tx.prl.tcpc.is_rx_enable_done()) { return No_State_Change; }

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

    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Layer_Reset_for_Transmit_State : public afsm::state<PRL_Tx, PRL_Tx_Layer_Reset_for_Transmit_State, PRL_Tx_Layer_Reset_for_Transmit> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& prl = prl_tx.prl;
        prl_tx.log_state();

        // NOTE: Spec says to reset only `msg_id_counter` here, and reset
        // `msg_id_stored` via RX state change. But etl::fsm does not re-run
        // `on_enter` if we come from current state to itself.
        // So, reset both here.
        prl.reset_msg_counters();
        // This will not make sense, because we do not send GoodCRC in software,
        // and every input packet causes returns to initial state immediately.
        // But this is left for consistency with the spec.
        prl.prl_rx.change_state(PRL_Rx_Wait_for_PHY_Message);

        return PRL_Tx_Construct_Message;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Construct_Message_State : public afsm::state<PRL_Tx, PRL_Tx_Construct_Message_State, PRL_Tx_Construct_Message> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tx_chunk.header.message_id = port.tx_msg_id_counter;
        port.tx_chunk.header.spec_revision = port.revision;

        // Here we should fill power/data roles. But since we are Sink-only UFP,
        // we can just use default values (zeroes).

        //
        // Prepare for sending (PRL_TX can be used without RCH/TCH).
        //

        // Block pending garbage from driver
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);

        // Reset PRL_TX "output"
        port.prl_tx_flags.clear(PRL_TX_FLAG::TX_COMPLETED);
        port.prl_tx_flags.clear(PRL_TX_FLAG::TX_ERROR);

        // Kick driver
        prl_tx.prl.tcpc.req_transmit();

        return PRL_Tx_Wait_for_PHY_Response;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

// Here we wait for "GoodCRC" or failure.
class PRL_Tx_Wait_for_PHY_Response_State : public afsm::state<PRL_Tx, PRL_Tx_Wait_for_PHY_Response_State, PRL_Tx_Wait_for_PHY_Response> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        prl_tx.log_state();

        // Timer should be used ONLY when hardware confirmation not supported
        // if (!prl_tx.prl.tcpc.get_hw_features().tx_auto_goodcrc_check) {
        //    prl_tx.prl.port.timers.start(PD_TIMEOUT::tReceive);
        // }
        return No_State_Change;
    }

    static auto on_run_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;

        auto status = port.tcpc_tx_status.load();

        if (status == TCPC_TRANSMIT_STATUS::SUCCEEDED) {
            return PRL_Tx_Match_MessageID;
        }

        if (status == TCPC_TRANSMIT_STATUS::FAILED) {
            return PRL_Tx_Check_RetryCounter;
        }

        // Actual only for software CRC processing
        // if (port.timers.is_expired(PD_TIMEOUT::tReceive)) {
        //    return PRL_Tx_Check_RetryCounter;
        // }

        return No_State_Change;
    }

    // static void on_exit_state(PRL_Tx& prl_tx) {
    //    prl_tx.prl.port.timers.stop(PD_TIMEOUT::tReceive);
    // }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Match_MessageID_State : public afsm::state<PRL_Tx, PRL_Tx_Match_MessageID_State, PRL_Tx_Match_MessageID> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        prl_tx.log_state();

        // Since message id match is currently embedded in
        // transfer success status, just forward to next state
        return PRL_Tx_Message_Sent;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Message_Sent_State : public afsm::state<PRL_Tx, PRL_Tx_Message_Sent_State, PRL_Tx_Message_Sent> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tx_msg_id_counter++;
        port.prl_tx_flags.set(PRL_TX_FLAG::TX_COMPLETED);

        // Ensure one more loop run, to invoke RCH/TCH after PRL_TX completed
        // TODO: can be removed if RCH/TCH FSMs are invoked after PRL_TX.
        prl_tx.prl.request_wakeup();

        return PRL_Tx_Wait_for_Message_Request;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Check_RetryCounter_State : public afsm::state<PRL_Tx, PRL_Tx_Check_RetryCounter_State, PRL_Tx_Check_RetryCounter> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Retries are NOT used:
        //
        // - for Cable Plug
        // - for Extended Message with data size > MaxExtendedMsgLegacyLen that
        //   has not been chunked
        //
        // Since we are Sink-only, without unchunked ext msg support - no extra
        // checks needed. Always use retries is supported by hardware.

        if (prl_tx.prl.tcpc.get_hw_features().tx_auto_retry) {
            // Don't try retransmit if supported by hardware.
            return PRL_Tx_Transmission_Error;
        }

        port.tx_retry_counter++;

        // TODO: check if retries count should depend on negotiated revision (2 or 3)
        if (port.tx_retry_counter > port.max_retries()) {
            return PRL_Tx_Transmission_Error;

        }
        return PRL_Tx_Construct_Message;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Transmission_Error_State : public afsm::state<PRL_Tx, PRL_Tx_Transmission_Error_State, PRL_Tx_Transmission_Error> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        port.tx_msg_id_counter++;
        // Don't report PE about error here, allow RCH/TCH to handle & care.
        port.prl_tx_flags.set(PRL_TX_FLAG::TX_ERROR);

        // Ensure one more loop run, to invoke RCH/TCH after PRL_TX completed
        // TODO: can be removed if RCH/TCH FSMs are invoked after PRL_TX.
        prl_tx.prl.request_wakeup();

        return PRL_Tx_Wait_for_Message_Request;
    }

    static auto on_run_state(PRL_Tx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Discard_Message_State : public afsm::state<PRL_Tx, PRL_Tx_Discard_Message_State, PRL_Tx_Discard_Message> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        prl_tx.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;

        // Do discard, if any TX chunk processing:
        // - input queued to send
        // - passed to driver, and sending in progress
        if (port.prl_tx_flags.test_and_clear(PRL_TX_FLAG::TX_CHUNK_ENQUEUED) ||
            is_tcpc_transmit_in_progress(port.tcpc_tx_status.load()))
        {
            port.tx_msg_id_counter++;
            prl_tx.prl.report_pe(MsgToPe_PrlReportDiscard{});
        }
        return PRL_Tx_PHY_Layer_Reset;
    }

    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Snk_Start_of_AMS_State : public afsm::state<PRL_Tx, PRL_Tx_Snk_Start_of_AMS_State, PRL_Tx_Snk_Start_of_AMS> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& port = prl_tx.prl.port;
        prl_tx.log_state();

        // Reuse existing event to switch state, if condition satisfied
        if (port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }

    static auto on_run_state(PRL_Tx& prl_tx) -> state_id_t {
        if (prl_tx.prl.port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED)) {
            return PRL_Tx_Snk_Pending;
        }
        return No_State_Change;
    }

    static void on_exit_state(PRL_Tx&) {}
};

class PRL_Tx_Snk_Pending_State : public afsm::state<PRL_Tx, PRL_Tx_Snk_Pending_State, PRL_Tx_Snk_Pending> {
public:
    static auto on_enter_state(PRL_Tx& prl_tx) -> state_id_t {
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

    static auto on_run_state(PRL_Tx& prl_tx) -> state_id_t {
        auto& prl = prl_tx.prl;
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

    static void on_exit_state(PRL_Tx& prl_tx) {
        prl_tx.prl.port.timers.stop(PD_TIMEOUT::tActiveCcPollingDebounce);
    }
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.3 Protocol Layer Message Reception

class PRL_Rx_Wait_for_PHY_Message_State : public afsm::state<PRL_Rx, PRL_Rx_Wait_for_PHY_Message_State, PRL_Rx_Wait_for_PHY_Message> {
public:
    static auto on_enter_state(PRL_Rx& prl_rx) -> state_id_t {
        prl_rx.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_Rx& prl_rx) -> state_id_t {
        auto& prl = prl_rx.prl;
        auto& port = prl.port;

        if (port.prl_rch_flags.test(RCH_FLAG::RX_ENQUEUED)) {
            // In theory, we can have pending packet in RCH, re-routed by
            // discard in TCH. Postpone processing of new one to next cycle,
            // to allow RCH to finish.
            //
            // This is not expected to happen, because we do multiple RCH/TCH
            // calls.
            prl.request_wakeup();
            return No_State_Change;
        }

        if (!prl.tcpc.fetch_rx_data()) { return No_State_Change; }

        if (port.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
            return PRL_Rx_Layer_Reset_for_Receive;
        }

        return PRL_Rx_Send_GoodCRC;
    }

    static void on_exit_state(PRL_Rx&) {}
};

class PRL_Rx_Layer_Reset_for_Receive_State : public afsm::state<PRL_Rx, PRL_Rx_Layer_Reset_for_Receive_State, PRL_Rx_Layer_Reset_for_Receive> {
public:
    static auto on_enter_state(PRL_Rx& prl_rx) -> state_id_t {
        auto& prl = prl_rx.prl;
        auto& port = prl.port;
        prl_rx.log_state();

        // Similar to init, but skip RX and (?) revision clear.
        prl.prl_rch.change_state(afsm::Uninitialized);
        prl.prl_tch.change_state(afsm::Uninitialized);
        prl.prl_tx.change_state(afsm::Uninitialized);

        port.prl_tx_flags.clear_all();
        port.prl_rch_flags.clear_all();
        port.prl_tch_flags.clear_all();

        prl.reset_msg_counters();

        prl.prl_rch.change_state(RCH_Wait_For_Message_From_Protocol_Layer);
        prl.prl_tch.change_state(TCH_Wait_For_Message_Request_From_Policy_Engine);
        prl.prl_tx.change_state(PRL_Tx_PHY_Layer_Reset);

        prl.report_pe(MsgToPe_PrlSoftResetFromPartner{});
        return PRL_Rx_Send_GoodCRC;
    }

    static auto on_run_state(PRL_Rx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Rx&) {}
};

class PRL_Rx_Send_GoodCRC_State : public afsm::state<PRL_Rx, PRL_Rx_Send_GoodCRC_State, PRL_Rx_Send_GoodCRC> {
public:
    // All modern hardware sends CRC automatically. This state exists
    // to match spec and for potential extensions.
    static auto on_enter_state(PRL_Rx& prl_rx) -> state_id_t {
        prl_rx.log_state();
        return PRL_Rx_Check_MessageID;
    }

    static auto on_run_state(PRL_Rx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Rx&) {}
};

class PRL_Rx_Check_MessageID_State : public afsm::state<PRL_Rx, PRL_Rx_Check_MessageID_State, PRL_Rx_Check_MessageID> {
public:
    static auto on_enter_state(PRL_Rx& prl_rx) -> state_id_t {
        auto& port = prl_rx.prl.port;
        prl_rx.log_state();

        if (port.rx_msg_id_stored == port.rx_chunk.header.message_id) {
            // Ignore duplicated
            return PRL_Rx_Wait_for_PHY_Message;
        }
        return PRL_Rx_Store_MessageID;
    }

    static auto on_run_state(PRL_Rx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Rx&) {}
};

class PRL_Rx_Store_MessageID_State : public afsm::state<PRL_Rx, PRL_Rx_Store_MessageID_State, PRL_Rx_Store_MessageID> {
public:
    static auto on_enter_state(PRL_Rx& prl_rx) -> state_id_t {
        auto& port = prl_rx.prl.port;
        auto& prl = prl_rx.prl;
        prl_rx.log_state();

        port.rx_msg_id_stored = port.rx_chunk.header.message_id;

        // Rev 3.2 says ping is deprecated => Ignore it completely
        // (it should not discard, affect chunking and so on).
        if (port.rx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Ping)) { return PRL_Rx_Wait_for_PHY_Message; }

        // Discard TX if:
        //
        // - new data enqueued (but not sent yet)
        // - sending in progress
        // - failed
        //
        // Don't discard if sending succeeded. Let it finish as usual, because
        // this status can arrive together with new incoming message.
        //
        auto status = port.tcpc_tx_status.load();
        if ((status != TCPC_TRANSMIT_STATUS::UNSET && status != TCPC_TRANSMIT_STATUS::SUCCEEDED) ||
            port.prl_tx_flags.test(PRL_TX_FLAG::TX_CHUNK_ENQUEUED))
        {
            prl.prl_tx.change_state(PRL_Tx_Discard_Message);
        }

        // [rev3.2] 6.12.2.1.4 Chunked Message Router State Diagram
        //
        // Route message to RCH/TCH. Since RTR has no stored states, it is
        // more simple to embed its logic here.

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

    static auto on_run_state(PRL_Rx&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_Rx&) {}
};


////////////////////////////////////////////////////////////////////////////////
// [rev3.2] 6.12.2.4 Hard Reset operation

class PRL_HR_IDLE_State : public afsm::state<PRL_HR, PRL_HR_IDLE_State, PRL_HR_IDLE> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        hr.prl.port.prl_hr_flags.clear_all();
        return No_State_Change;
    }

    static auto on_run_state(PRL_HR& hr) -> state_id_t {
        auto& port = hr.prl.port;

        if (port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER) ||
            port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PE))
        {
            return PRL_HR_Reset_Layer;
        }
        return No_State_Change;
    }

    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_Reset_Layer_State : public afsm::state<PRL_HR, PRL_HR_Reset_Layer_State, PRL_HR_Reset_Layer> {
public:


    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        hr.prl.port.revision = MaxSupportedRevision;

        // Start with RX path disable (and FIFO clear).
        hr.prl.tcpc.req_rx_enable(false);

        return No_State_Change;
    }

    static auto on_run_state(PRL_HR& hr) -> state_id_t {
        auto& prl = hr.prl;

        // Wait for TCPC operation complete
        if (!prl.tcpc.is_rx_enable_done()) { return No_State_Change; }

        // Route state, depending on hard reset type requested
        if (prl.port.prl_hr_flags.test(PRL_HR_FLAG::HARD_RESET_FROM_PARTNER)) {
            return PRL_HR_Indicate_Hard_Reset;
        }
        return PRL_HR_Request_Hard_Reset;
    }

    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_Indicate_Hard_Reset_State : public afsm::state<PRL_HR, PRL_HR_Indicate_Hard_Reset_State, PRL_HR_Indicate_Hard_Reset> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        hr.prl.report_pe(MsgToPe_PrlHardResetFromPartner{});
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }

    static auto on_run_state(PRL_HR&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_Request_Hard_Reset_State : public afsm::state<PRL_HR, PRL_HR_Request_Hard_Reset_State, PRL_HR_Request_Hard_Reset> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        hr.prl.tcpc.req_hr_send();
        return No_State_Change;
    }

    static auto on_run_state(PRL_HR& hr) -> state_id_t {
        auto& prl = hr.prl;

        // Wait for TCPC call to complete. This does NOT mean transfer ended.
        // This means driver accepted request and commanded chip to send HR.
        // Final result is available via `port.tcpc_tx_status` (as for ordinary
        // transfer)
        if (!prl.tcpc.is_hr_send_done()) { return No_State_Change; }

        return PRL_HR_Wait_for_PHY_Hard_Reset_Complete;
    }

    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State : public afsm::state<PRL_HR, PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State, PRL_HR_Wait_for_PHY_Hard_Reset_Complete> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        auto& port = hr.prl.port;
        hr.log_state();

        port.timers.start(PD_TIMEOUT::tHardResetComplete);
        return No_State_Change;
    }

    static auto on_run_state(PRL_HR& hr) -> state_id_t {
        auto& port = hr.prl.port;
        auto status = port.tcpc_tx_status.load();

        if ((status == TCPC_TRANSMIT_STATUS::SUCCEEDED) ||
            (status == TCPC_TRANSMIT_STATUS::FAILED))
        {
            if (status == TCPC_TRANSMIT_STATUS::FAILED) {
                PRL_LOGE("Hard Reset sending failed");
            }
            return PRL_HR_PHY_Hard_Reset_Requested;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tHardResetComplete)) {
            PRL_LOGE("Hard Reset sending timed out");
            return PRL_HR_PHY_Hard_Reset_Requested;
        }

        return No_State_Change;
    }

    static void on_exit_state(PRL_HR& hr) {
        hr.prl.port.timers.stop(PD_TIMEOUT::tHardResetComplete);
    }
};

class PRL_HR_PHY_Hard_Reset_Requested_State : public afsm::state<PRL_HR, PRL_HR_PHY_Hard_Reset_Requested_State, PRL_HR_PHY_Hard_Reset_Requested> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        hr.prl.report_pe(MsgToPe_PrlHardResetSent{});
        return PRL_HR_Wait_for_PE_Hard_Reset_Complete;
    }

    static auto on_run_state(PRL_HR&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_Wait_for_PE_Hard_Reset_Complete_State : public afsm::state<PRL_HR, PRL_HR_Wait_for_PE_Hard_Reset_Complete_State, PRL_HR_Wait_for_PE_Hard_Reset_Complete> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PRL_HR& hr) -> state_id_t {
        //
        // 6.12.2.4.7 PRL_HR_PE_Hard_Reset_Complete
        // If Hard Reset Signaling is still pending due to a non-Idle channel
        // this Shall be cleared and not sent
        //
        // TODO: fusb302 has no way to interrupt pending HR. We rely on
        // chip/timer timeouts. May be, driver API should be extended
        // for another hardware.
        //

        if (hr.prl.port.prl_hr_flags.test_and_clear(PRL_HR_FLAG::PE_HARD_RESET_COMPLETE)) {
            return PRL_HR_PE_Hard_Reset_Complete;
        }
        return No_State_Change;
    }

    static void on_exit_state(PRL_HR&) {}
};

class PRL_HR_PE_Hard_Reset_Complete_State : public afsm::state<PRL_HR, PRL_HR_PE_Hard_Reset_Complete_State, PRL_HR_PE_Hard_Reset_Complete> {
public:
    static auto on_enter_state(PRL_HR& hr) -> state_id_t {
        hr.log_state();

        return PRL_HR_IDLE;
    }

    static auto on_run_state(PRL_HR&) -> state_id_t { return No_State_Change; }
    static void on_exit_state(PRL_HR&) {}
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
    port.prl_rtr = &prl_event_listener;
}

void PRL::init() {
    PRL_LOGI("PRL init begin");

    prl_hr.change_state(afsm::Uninitialized);
    port.prl_hr_flags.clear_all();
    prl_hr.change_state(PRL_HR_IDLE);

    prl_rch.change_state(afsm::Uninitialized);
    prl_tch.change_state(afsm::Uninitialized);
    prl_rx.change_state(afsm::Uninitialized);
    prl_tx.change_state(afsm::Uninitialized);

    port.prl_tx_flags.clear_all();
    port.prl_rch_flags.clear_all();
    port.prl_tch_flags.clear_all();
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);

    port.timers.stop_range(PD_TIMERS_RANGE::PRL);

    // NOTE: negotiated revision stays intact. It's cleared via PE init and
    // hard reset.
    reset_msg_counters();

    prl_rch.change_state(RCH_Wait_For_Message_From_Protocol_Layer);
    prl_tch.change_state(TCH_Wait_For_Message_Request_From_Policy_Engine);
    prl_rx.change_state(PRL_Rx_Wait_for_PHY_Message);
    // Reset TX last, because it does driver call on init.
    prl_tx.change_state(PRL_Tx_PHY_Layer_Reset);
    // Ensure loop repeat to continue PE States, which wait for PRL run.
    request_wakeup();

    PRL_LOGI("PRL init end");
}

void PRL::report_pe(const etl::imessage& msg) {
    port.notify_pe(msg);
    request_wakeup();
}

void PRL::reset_msg_counters() {
    port.rx_msg_id_stored = -1;
    port.tx_msg_id_counter = 0;
}

void PRL::prl_tx_enquire_chunk() {
    // Ensure to prohibit accepting statuses from driver
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);

    // Clear PRL_TX "output"
    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_COMPLETED);
    port.prl_tx_flags.clear(PRL_TX_FLAG::TX_ERROR);

    // Mark tx_chunk ready to be sent
    port.prl_tx_flags.set(PRL_TX_FLAG::TX_CHUNK_ENQUEUED);

    request_wakeup();
}


////////////////////////////////////////////////////////////////////////////////
// FSMs configs

using RCH_STATES = afsm::state_pack<
    RCH_Wait_For_Message_From_Protocol_Layer_State,
    RCH_Pass_Up_Message_State,
    RCH_Processing_Extended_Message_State,
    RCH_Requesting_Chunk_State,
    RCH_Waiting_Chunk_State,
    RCH_Report_Error_State
>;

PRL_RCH::PRL_RCH(PRL& prl) : prl{prl} {
    set_states<RCH_STATES>();
};

void PRL_RCH::log_state() const {
    PRL_LOGI("PRL_RCH state => {}", prl_rch_state_to_desc(get_state_id()));
}

using TCH_STATES = afsm::state_pack<
    TCH_Wait_For_Message_Request_From_Policy_Engine_State,
    TCH_Pass_Down_Message_State,
    TCH_Wait_For_Transmission_Complete_State,
    TCH_Message_Sent_State,
    TCH_Prepare_To_Send_Chunked_Message_State,
    TCH_Construct_Chunked_Message_State,
    TCH_Sending_Chunked_Message_State,
    TCH_Wait_Chunk_Request_State,
    TCH_Message_Received_State,
    TCH_Report_Error_State
>;

PRL_TCH::PRL_TCH(PRL& prl) : prl{prl} {
    set_states<TCH_STATES>();
}

void PRL_TCH::log_state() const {
    PRL_LOGI("PRL_TCH state => {}", prl_tch_state_to_desc(get_state_id()));
}

using PRL_TX_STATES = afsm::state_pack<
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
>;

PRL_Tx::PRL_Tx(PRL& prl) : prl{prl} {
    set_states<PRL_TX_STATES>();
}

void PRL_Tx::log_state() const {
    PRL_LOGI("PRL_Tx state => {}", prl_tx_state_to_desc(get_state_id()));
}

using PRL_RX_STATES = afsm::state_pack<
    PRL_Rx_Wait_for_PHY_Message_State,
    PRL_Rx_Layer_Reset_for_Receive_State,
    PRL_Rx_Send_GoodCRC_State,
    PRL_Rx_Check_MessageID_State,
    PRL_Rx_Store_MessageID_State
>;

PRL_Rx::PRL_Rx(PRL& prl) : prl{prl} {
    set_states<PRL_RX_STATES>();
}

void PRL_Rx::log_state() const {
    PRL_LOGI("PRL_Rx state => {}", prl_rx_state_to_desc(get_state_id()));
}

using PRL_HR_STATES = afsm::state_pack<
    PRL_HR_IDLE_State,
    PRL_HR_Reset_Layer_State,
    PRL_HR_Indicate_Hard_Reset_State,
    PRL_HR_Request_Hard_Reset_State,
    PRL_HR_Wait_for_PHY_Hard_Reset_Complete_State,
    PRL_HR_PHY_Hard_Reset_Requested_State,
    PRL_HR_Wait_for_PE_Hard_Reset_Complete_State,
    PRL_HR_PE_Hard_Reset_Complete_State
>;

PRL_HR::PRL_HR(PRL& prl) : prl{prl} {
    set_states<PRL_HR_STATES>();
}

void PRL_HR::log_state() const {
    PRL_LOGI("PRL_HR state => {}", prl_hr_state_to_desc(get_state_id()));
}


void PRL_EventListener::on_receive(const MsgSysUpdate&) {
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

            prl.prl_hr.run();

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
                prl.prl_tx.run();
            }

            prl.prl_rx.run();
            prl.prl_rch.run();
            // First TCH call needed when PE enqueued message, to start
            // chunking / transfer.
            prl.prl_tch.run();
            prl.prl_tx.run();

            //
            // Repeat TCH/RCH calls for quick-consume previous changes
            //

            // After transfer complete - PE should be notified, call TCH again.
            prl.prl_tch.run();
            // Once more to catch edge case
            prl.prl_tch.run();
            // Repeat RCH call to land
            // - re-routed TCH message
            // - prl_tx status update after chunk request
            prl.prl_rch.run();
            break;
    }

    if (prl.has_deferred_wakeup_request) {
        prl.has_deferred_wakeup_request = false;
        prl.request_wakeup();
    }
}

void PRL_EventListener::on_receive(const MsgToPrl_EnquireRestart&) {
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
}

void PRL_EventListener::on_receive(const MsgToPrl_CtlMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    prl.port.tx_emsg.clear();
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
}

void PRL_EventListener::on_receive(const MsgToPrl_DataMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
}

void PRL_EventListener::on_receive(const MsgToPrl_ExtMsgFromPe& msg) {
    PD_HEADER hdr{};
    hdr.message_type = msg.type;
    hdr.extended = 1;
    prl.port.tx_emsg.header = hdr;

    prl.port.prl_tch_flags.set(TCH_FLAG::MSG_FROM_PE_ENQUEUED);
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
