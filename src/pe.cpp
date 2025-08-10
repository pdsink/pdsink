#include <etl/algorithm.h>
#include <etl/array.h>

#include "common_macros.h"
#include "dpm.h"
#include "idriver.h"
#include "pd_log.h"
#include "pe.h"
#include "port.h"
#include "utils/etl_state_pack.h"

namespace pd {

enum PE_State {
    // 8.3.3.3 Policy Engine Sink Port State Diagram
    PE_SNK_Startup,
    PE_SNK_Discovery,
    PE_SNK_Wait_for_Capabilities,
    PE_SNK_Evaluate_Capability,
    PE_SNK_Select_Capability,
    PE_SNK_Transition_Sink,
    PE_SNK_Ready,

    PE_SNK_Give_Sink_Cap,

    PE_SNK_EPR_Keep_Alive,
    PE_SNK_Hard_Reset,
    PE_SNK_Transition_to_default,

    // [rev3.2] 8.3.3.4.2 SOP Sink Port Soft Reset and Protocol Error State Diagram
    PE_SNK_Soft_Reset,
    PE_SNK_Send_Soft_Reset,

    // [rev3.2] 8.3.3.6.2 Sink Port Not Supported Message State Diagram
    PE_SNK_Send_Not_Supported,

    // [rev3.2] 8.3.3.7.2.1 PE_SNK_Source_Alert_Received State
    PE_SNK_Source_Alert_Received,

    // [rev3.2] 8.3.3.26.2 Sink EPR Mode Entry State Diagram
    PE_SNK_Send_EPR_Mode_Entry,
    PE_SNK_EPR_Mode_Entry_Wait_For_Response,
    // [rev3.2] 8.3.3.26.4 Sink EPR Mode Exit State Diagram
    PE_SNK_EPR_Mode_Exit_Received, // Manual exit not needed, only SRC-forced

    // [rev3.2] 8.3.3.27 BIST State Diagrams
    PE_BIST_Activate, // Not in spec, common entry point
    PE_BIST_Carrier_Mode,
    PE_BIST_Test_Mode,

    // [rev3.2] 8.3.3.15.2 Give Revision State Diagram
    PE_Give_Revision,

    // 8.3.3.2.7 PE_SRC_Disabled State
    PE_Src_Disabled,
};

namespace {
    constexpr auto pe_state_to_desc(int state) -> const char* {
        switch (state) {
            case PE_SNK_Startup: return "PE_SNK_Startup";
            case PE_SNK_Discovery: return "PE_SNK_Discovery";
            case PE_SNK_Wait_for_Capabilities: return "PE_SNK_Wait_for_Capabilities";
            case PE_SNK_Evaluate_Capability: return "PE_SNK_Evaluate_Capability";
            case PE_SNK_Select_Capability: return "PE_SNK_Select_Capability";
            case PE_SNK_Transition_Sink: return "PE_SNK_Transition_Sink";
            case PE_SNK_Ready: return "PE_SNK_Ready";
            case PE_SNK_Give_Sink_Cap: return "PE_SNK_Give_Sink_Cap";
            case PE_SNK_EPR_Keep_Alive: return "PE_SNK_EPR_Keep_Alive";
            case PE_SNK_Hard_Reset: return "PE_SNK_Hard_Reset";
            case PE_SNK_Transition_to_default: return "PE_SNK_Transition_to_default";
            case PE_SNK_Soft_Reset: return "PE_SNK_Soft_Reset";
            case PE_SNK_Send_Soft_Reset: return "PE_SNK_Send_Soft_Reset";
            case PE_SNK_Send_Not_Supported: return "PE_SNK_Send_Not_Supported";
            case PE_SNK_Source_Alert_Received: return "PE_SNK_Source_Alert_Received";
            case PE_SNK_Send_EPR_Mode_Entry: return "PE_SNK_Send_EPR_Mode_Entry";
            case PE_SNK_EPR_Mode_Entry_Wait_For_Response: return "PE_SNK_EPR_Mode_Entry_Wait_For_Response";
            case PE_SNK_EPR_Mode_Exit_Received: return "PE_SNK_EPR_Mode_Exit_Received";
            case PE_BIST_Activate: return "PE_BIST_Activate";
            case PE_BIST_Carrier_Mode: return "PE_BIST_Carrier_Mode";
            case PE_BIST_Test_Mode: return "PE_BIST_Test_Mode";
            case PE_Give_Revision: return "PE_Give_Revision";
            case PE_Src_Disabled: return "PE_Src_Disabled";
            default: return "Unknown PE state";
        }
    }
} // namespace


//
// Macros to quickly create common methods
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

#define ON_EVENT_NOTHING \
auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t { \
    return No_State_Change; \
}

#define ON_TRANSIT_TO \
auto on_event(const MsgTransitTo& event) -> etl::fsm_state_id_t { \
    return event.state_id; \
}

class PE_SNK_Startup_State : public etl::fsm_state<PE, PE_SNK_Startup_State, PE_SNK_Startup, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        port.notify_prl(MsgToPrl_SoftResetFromPe{});
        port.pe_flags.clear(PE_FLAG::HAS_EXPLICIT_CONTRACT);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (!port.is_prl_running()) { return No_State_Change; }
        return PE_SNK_Discovery;
    }
};


class PE_SNK_Discovery_State : public etl::fsm_state<PE, PE_SNK_Discovery_State, PE_SNK_Discovery, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        // As a Sink, we detect TC attach via CC1/CC2 with debounce. VBUS should
        // be stable at this moment, so there is no need to wait.
        return PE_SNK_Wait_for_Capabilities;
    }
};


class PE_SNK_Wait_for_Capabilities_State : public etl::fsm_state<PE, PE_SNK_Wait_for_Capabilities_State, PE_SNK_Wait_for_Capabilities, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.port.timers.start(PD_TIMEOUT::tTypeCSinkWaitCap);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            // The spec requires an exact match of the capabilities type and the
            // current sink mode to accept.
            if (pe.is_in_epr_mode()) {
                if (port.rx_emsg.is_ext_msg(PD_EXT_MSGT::Source_Capabilities_Extended)) {
                    return PE_SNK_Evaluate_Capability;
                }
            } else {
                if (port.rx_emsg.is_data_msg(PD_DATA_MSGT::Source_Capabilities)) {
                    return PE_SNK_Evaluate_Capability;
                }
            }
        }

        if (port.timers.is_expired(PD_TIMEOUT::tTypeCSinkWaitCap)) {
            port.pe_flags.set(PE_FLAG::HR_BY_CAPS_TIMEOUT);
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        get_fsm_context().port.timers.stop(PD_TIMEOUT::tTypeCSinkWaitCap);
    }
};


class PE_SNK_Evaluate_Capability_State : public etl::fsm_state<PE, PE_SNK_Evaluate_Capability_State, PE_SNK_Evaluate_Capability, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        port.hard_reset_counter = 0;
        port.revision = static_cast<PD_REVISION::Type>(
            etl::min(static_cast<uint16_t>(PD_REVISION::REV30), port.rx_emsg.header.spec_revision));

        port.source_caps.clear();
        for (int i = 0; i < port.rx_emsg.size_to_pdo_count(); i++) {
            port.source_caps.push_back(port.rx_emsg.read32(i*4));
        }

        port.pe_flags.set(PE_FLAG::IS_FROM_EVALUATE_CAPABILITY);
        port.notify_dpm(MsgToDpm_SrcCapsReceived());
        return PE_SNK_Select_Capability;
    }
};

//
// This is the main place where the explicit contract is established or changed.
// We come here in these cases:
//
// 1. Initially, after receiving a Source_Capabilities message.
// 2. After upgrading to EPR and receiving an EPR_Source_Capabilities message.
// 3. In PPS mode after a timeout.
// 4. After the DPM requests a contract change.
//
// This state requests the desired RDO from the DPM, sends it to the Source,
// and waits for confirmation. If the SRC asks to WAIT, go to the READY state
// (it will retry after a delay).
//
// After success, if the SRC supports EPR and we are NOT in EPR mode => force
// an upgrade. This upgrade is not part of the PD spec, but for a sink-only
// device this is a good place to keep things simple.
//

class PE_SNK_Select_Capability_State : public etl::fsm_state<PE, PE_SNK_Select_Capability_State, PE_SNK_Select_Capability, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        auto rdo_and_pdo = pe.dpm.get_request_data_object(port.source_caps);

        // This is not needed, but it exists to suppress warnings from code checkers.
        if (!rdo_and_pdo.first) {
            PE_LOGE("Bad RDO from DPM (zero)");
            return PE_SNK_Hard_Reset;
        }

        // Prepare & send request, depending on SPR/EPR mode
        port.tx_emsg.clear();

        // Remember the RDO to store after success
        port.rdo_to_request = rdo_and_pdo.first;

        if (pe.is_in_epr_mode()) {
            port.tx_emsg.append32(rdo_and_pdo.first);
            port.tx_emsg.append32(rdo_and_pdo.second);
            pe.send_data_msg(PD_DATA_MSGT::EPR_Request);
        } else {
            port.tx_emsg.append32(rdo_and_pdo.first);
            pe.send_data_msg(PD_DATA_MSGT::Request);
        }

        pe.check_request_progress_enter();
        port.timers.stop(PD_TIMEOUT::tSinkRequest);

        // Don't break on error; process errors manually.
        port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        auto send_status = pe.check_request_progress_run();

        // Reproduce AMS interrupt logic:
        // - If this state is a standalone request (DPM), roll back to the Ready
        //   state. DPM means an explicit contract already exists.
        // - If we came from Evaluate_Capability and the AMS was interrupted after
        //   the first message => perform a Soft Reset.
        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            if (port.pe_flags.test(PE_FLAG::IS_FROM_EVALUATE_CAPABILITY)) {
                return PE_SNK_Send_Soft_Reset;
            }
            return PE_SNK_Ready;
        }
        if (send_status == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = port.rx_emsg;

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Accept))
            {
                bool is_first_contract = !port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT);

                port.pe_flags.set(PE_FLAG::HAS_EXPLICIT_CONTRACT);
                port.rdo_contracted = port.rdo_to_request;

                if (is_first_contract && (pe.is_in_epr_mode() || !pe.is_epr_mode_available())) {
                    // Report handshake complete if this is the first contract
                    // and we should not try EPR
                    port.notify_dpm(MsgToDpm_HandshakeDone());
                }

                if (port.dpm_requests.test_and_clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                    port.notify_dpm(MsgToDpm_NewPowerLevel(true));
                }

                port.notify_dpm(MsgToDpm_SelectCapDone());
                return PE_SNK_Transition_Sink;
            }

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Wait))
            {
                if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
                    // The spec requires initializing this timer on PE_SNK_Ready entry,
                    // but it is more convenient to do it here.
                    port.timers.start(PD_TIMEOUT::tSinkRequest);
                    return PE_SNK_Ready;
                }
                return PE_SNK_Wait_for_Capabilities;
            }

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Reject))
            {
                if (port.dpm_requests.test_and_clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                    port.notify_dpm(MsgToDpm_NewPowerLevel(false));
                }

                if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
                    return PE_SNK_Ready;
                }
                return PE_SNK_Wait_for_Capabilities;
            }

            // Anything not expected => soft reset
            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Hard_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.port.pe_flags.clear(PE_FLAG::IS_FROM_EVALUATE_CAPABILITY);
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
        pe.check_request_progress_exit();
    }
};


class PE_SNK_Transition_Sink_State : public etl::fsm_state<PE, PE_SNK_Transition_Sink_State, PE_SNK_Transition_Sink, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        // There are two timeouts, depending on EPR mode. Both use the same
        // timer. Use the proper one for setup, but the SPR one for clear/check
        // (because clear/check use the same timer ID).
        if (port.pe_flags.test(PE_FLAG::IN_EPR_MODE)) {
            port.timers.start(PD_TIMEOUT::tPSTransition_EPR);
        } else {
            port.timers.start(PD_TIMEOUT::tPSTransition_SPR);
        }

        // Any PRL error at this stage should cause a hard reset.
        port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            if (port.rx_emsg.is_ctrl_msg(PD_CTRL_MSGT::PS_RDY)) {
                return PE_SNK_Ready;
            }
            // Anything else - protocol error
            return PE_SNK_Hard_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tPSTransition_SPR)) {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().port;
        port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
        port.timers.stop(PD_TIMEOUT::tPSTransition_SPR);
    }
};


class PE_SNK_Ready_State : public etl::fsm_state<PE, PE_SNK_Ready_State, PE_SNK_Ready, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::IS_FROM_EPR_KEEP_ALIVE)) {
            // Log returning from EPR Keep-Alive at debug level to reduce noise
            PE_LOGD("PE state => {}", pe_state_to_desc(pe.get_state_id()));
        } else {
            pe.log_state();
        }

        // Ensure flags from the previous send attempt are cleared.
        // If the sink returned to this state, everything starts from scratch.
        port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);
        port.pe_flags.clear(PE_FLAG::AMS_ACTIVE);
        port.pe_flags.clear(PE_FLAG::AMS_FIRST_MSG_SENT);

        if (pe.is_in_epr_mode()) {
            // If we are in EPR mode, re-arm the timer for an EPR Keep-Alive request
            port.timers.start(PD_TIMEOUT::tSinkEPRKeepAlive);
        } else {
            // Force entering EPR mode if possible
            if (pe.is_epr_mode_available()) {
                port.dpm_requests.set(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
            }
        }

        if (pe.is_in_pps_contract()) {
            // The PPS contract should be refreshed at least every 10 s
            // of inactivity. We use 5 s to be safe.
            port.timers.start(PD_TIMEOUT::tPPSRequest);
        }

        // Ensure we run after entering the state to process pending items (DPM requests)
        port.wakeup();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        bool sr_on_unsupported = port.pe_flags.test(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            auto& msg = port.rx_emsg;
            auto hdr = msg.header;

            if (hdr.extended > 0) {
                //
                // Extended message
                //
                switch (hdr.message_type)
                {
                case PD_EXT_MSGT::Source_Capabilities_Extended:
                    // If not in EPR mode, ignore this message
                    if (pe.is_in_epr_mode()) { return PE_SNK_Evaluate_Capability; }
                    break;

                case PD_EXT_MSGT::Extended_Control:
                    if (msg.is_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Get_Sink_Cap)) {
                        return PE_SNK_Give_Sink_Cap;
                    }
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;

                default:
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }
            }
            else if (hdr.data_obj_count > 0) {
                //
                // Data message (not extended + data objects exist)
                //
                switch (hdr.message_type)
                {
                case PD_DATA_MSGT::Source_Capabilities:
                    if (pe.is_in_epr_mode()) { return PE_SNK_Hard_Reset;}
                    return PE_SNK_Evaluate_Capability;

                case PD_DATA_MSGT::Vendor_Defined:
                    // No VDM support. Reject for PD 3.0+, and ignore for 2.0
                    if (port.revision >= PD_REVISION::REV30) { return PE_SNK_Send_Not_Supported; }
                    break;

                case PD_DATA_MSGT::BIST:
                    return PE_BIST_Activate;

                case PD_DATA_MSGT::Alert:
                    return PE_SNK_Source_Alert_Received;

                case PD_DATA_MSGT::EPR_Mode: {
                    // SRC requested to exit EPR mode (should not happen, but
                    // it's allowed by the spec)
                    EPRMDO eprmdo{msg.read32(0)};
                    if (eprmdo.action == EPR_MODE_ACTION::EXIT) {
                        return PE_SNK_EPR_Mode_Exit_Received;
                    }
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }

                default:
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }
            } else {
                //
                // Control message (not extended, no data objects)
                //
                switch (hdr.message_type)
                {
                case PD_CTRL_MSGT::GoodCRC: // Nothing to do, ignoring
                    break;

                case PD_CTRL_MSGT::GotoMin: // Deprecated as "Not Supported"
                    return PE_SNK_Send_Not_Supported;

                // Unexpected => soft reset
                case PD_CTRL_MSGT::Accept:
                case PD_CTRL_MSGT::Reject:
                    return PE_SNK_Send_Soft_Reset;

                case PD_CTRL_MSGT::Ping: // Deprecated as "ignored"
                    break;

                case PD_CTRL_MSGT::PS_RDY: // Unexpected => soft reset
                    return PE_SNK_Send_Soft_Reset;

                case PD_CTRL_MSGT::Get_Sink_Cap:
                    return PE_SNK_Give_Sink_Cap;

                case PD_CTRL_MSGT::Wait: // Unexpected => soft reset
                    return PE_SNK_Send_Soft_Reset;

                case PD_CTRL_MSGT::Not_Supported:
                    // Cannot be initiated by the SRC, but can be a reply after
                    // an interrupted AMS. So do nothing; just ignore the garbage
                    // to avoid an infinite ping-pong.
                    break;

                case PD_CTRL_MSGT::Get_Revision:
                    return PE_Give_Revision;

                default:
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }
            }
        }

        if (port.is_prl_busy()) { return No_State_Change; }

        // Special case: process a postponed Source Capabilities request.
        // If pending, don't try the DPM requests queue.
        if (!port.timers.is_disabled(PD_TIMEOUT::tSinkRequest))
        {
            if (port.timers.is_expired(PD_TIMEOUT::tSinkRequest)) {
                port.timers.stop(PD_TIMEOUT::tSinkRequest);
                return PE_SNK_Select_Capability;
            }
        }
        else
        {
            //
            // Process DPM requests
            //
            // NOTE: Request flags are cleared inside states when the result is
            // determined (success or failure). Any interruption of the process
            // leaves the request armed. Should be OK for the sink. Can be changed later.
            //

            port.pe_flags.set(PE_FLAG::AMS_ACTIVE);

            if (port.dpm_requests.test(DPM_REQUEST_FLAG::EPR_MODE_ENTRY)) {
                if (pe.is_in_epr_mode()) {
                    PE_LOGI("EPR mode entry requested, but already in EPR mode");
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
                } else if (!pe.is_epr_mode_available()) {
                    PE_LOGI("EPR mode entry requested, but not allowed");
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
                } else {
                    return PE_SNK_Send_EPR_Mode_Entry;
                }
            }

            if (port.dpm_requests.test(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                return PE_SNK_Select_Capability;
            }

            //
            // Add more DPM requests here if needed.
            //

            port.pe_flags.clear(PE_FLAG::AMS_ACTIVE);
        }

        //
        // Keep-alive for EPR mode / PPS contract
        //

        if (port.timers.is_expired(PD_TIMEOUT::tSinkEPRKeepAlive)) {
            return PE_SNK_EPR_Keep_Alive;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tPPSRequest)) {
            return PE_SNK_Select_Capability;
        }

        // TODO: Consider remove. This causes unnecessary flood.
        // If the event caused no activity, emit Idle to the DPM
        // port.notify_dpm(MsgToDpm_Idle());

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().port;
        port.timers.stop(PD_TIMEOUT::tSinkEPRKeepAlive);
        port.timers.stop(PD_TIMEOUT::tPPSRequest);
        port.pe_flags.clear(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
    }
};


class PE_SNK_Give_Sink_Cap_State : public etl::fsm_state<PE, PE_SNK_Give_Sink_Cap_State, PE_SNK_Give_Sink_Cap, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        bool is_epr = port.rx_emsg.header.extended;
        port.tx_emsg.clear();

        // DPM is responsible for providing properly padded sink PDOs.
        auto caps = pe.dpm.get_sink_pdo_list();

        // Fill data; length depends on the request type
        for (int i = 0, max = caps.size(); i < max; i++) {
            auto pdo = caps[i];

            if (!is_epr) {
                // For a regular request (non-EPR) only 7 PDOs are allowed
                if (i >= MaxPdoObjects_SPR) { break; }
            }

            port.tx_emsg.append32(pdo);
        }

        if (!is_epr) {
            pe.send_data_msg(PD_DATA_MSGT::Sink_Capabilities);
        } else {
            pe.send_ext_msg(PD_EXT_MSGT::EPR_Sink_Capabilities);
        }
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }

        // No more checks - rely on standard error processing.
        return No_State_Change;
    }
};


class PE_SNK_EPR_Keep_Alive_State : public etl::fsm_state<PE, PE_SNK_EPR_Keep_Alive_State, PE_SNK_EPR_Keep_Alive, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        // Manually log as debug level, to reduce noise
        PE_LOGD("PE state => {}", pe_state_to_desc(pe.get_state_id()));


        ECDB ecdb{};
        ecdb.type = PD_EXT_CTRL_MSGT::EPR_KeepAlive;

        port.tx_emsg.clear();
        port.tx_emsg.append16(ecdb.raw_value);

        pe.send_ext_msg(PD_EXT_MSGT::Extended_Control);
        pe.check_request_progress_enter();

        port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            // If the message was discarded due to another activity => the connection
            // is OK, and a heartbeat is not needed. Consider it successful.
            port.pe_flags.set(PE_FLAG::IS_FROM_EPR_KEEP_ALIVE);
            return PE_SNK_Ready;
        }

        if (send_status == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (port.rx_emsg.is_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_KeepAlive_Ack)) {
                port.pe_flags.set(PE_FLAG::IS_FROM_EPR_KEEP_ALIVE);
                return PE_SNK_Ready;
            }

            PE_LOGE("Protocol error: unexpected message received [0x{:04x}]", port.rx_emsg.header.raw_value);
            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Hard_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.check_request_progress_exit();
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
    }
};


class PE_SNK_Hard_Reset_State : public etl::fsm_state<PE, PE_SNK_Hard_Reset_State, PE_SNK_Hard_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        if (port.pe_flags.test_and_clear(PE_FLAG::HR_BY_CAPS_TIMEOUT) &&
            port.hard_reset_counter > nHardResetCount)
        {
            return PE_Src_Disabled;
        }

        port.pe_flags.set(PE_FLAG::PRL_HARD_RESET_PENDING);
        port.notify_prl(MsgToPrl_HardResetFromPe{});
        port.hard_reset_counter++;

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test(PE_FLAG::PRL_HARD_RESET_PENDING)) {
            return No_State_Change;
        }
        return PE_SNK_Transition_to_default;
    }
};


class PE_SNK_Transition_to_default_State : public etl::fsm_state<PE, PE_SNK_Transition_to_default_State, PE_SNK_Transition_to_default, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        port.pe_flags.clear_all();
        port.dpm_requests.clear_all();

        // If you need to pend, call `wait_dpm_transit_to_default(true)` in the
        // event handler, and `wait_dpm_transit_to_default(false)` to continue.
        port.notify_dpm(MsgToDpm_TransitToDefault());
        port.wakeup();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (!port.pe_flags.test(PE_FLAG::WAIT_DPM_TRANSIT_TO_DEFAULT)) {
            port.notify_prl(MsgToPrl_PEHardResetDone{});
            return PE_SNK_Startup;
        }
        return No_State_Change;
    }
};


// Come here when a Soft Reset is received from the SRC
class PE_SNK_Soft_Reset_State : public etl::fsm_state<PE, PE_SNK_Soft_Reset_State, PE_SNK_Soft_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.send_ctrl_msg(PD_CTRL_MSGT::Accept);

        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Wait_for_Capabilities;
        }
        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_DISCARDED) ||
            port.pe_flags.test_and_clear(PE_FLAG::PROTOCOL_ERROR)) {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().port;
        port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
    }
};


class PE_SNK_Send_Soft_Reset_State : public etl::fsm_state<PE, PE_SNK_Send_Soft_Reset_State, PE_SNK_Send_Soft_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        // Clean up flags from previous operations
        port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);
        port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);

        // Set up error handling and initial state
        port.pe_flags.set(PE_FLAG::CAN_SEND_SOFT_RESET);
        port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);

        port.notify_prl(MsgToPrl_SoftResetFromPe{});
        pe.check_request_progress_enter();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        // Wait until the PRL layer is ready
        if (!port.is_prl_running()) { return No_State_Change; }

        // Send only once per state entry
        if (port.pe_flags.test_and_clear(PE_FLAG::CAN_SEND_SOFT_RESET)) {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Soft_Reset);
            return No_State_Change;
        }

        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            return PE_SNK_Ready;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (port.rx_emsg.is_ctrl_msg(PD_CTRL_MSGT::Accept)) {
                return PE_SNK_Wait_for_Capabilities;
            }
        }

        if (port.pe_flags.test_and_clear(PE_FLAG::PROTOCOL_ERROR) ||
            port.timers.is_expired(PD_TIMEOUT::tSenderResponse))
        {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
        pe.check_request_progress_exit();
    }
};


class PE_SNK_Send_Not_Supported_State : public etl::fsm_state<PE, PE_SNK_Send_Not_Supported_State, PE_SNK_Send_Not_Supported, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        // The reply depends on the PD revision. For PD 3.0+, use Not_Supported;
        // otherwise use Reject.
        if (pe.port.revision < PD_REVISION::REV30) {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Reject);
        } else {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Not_Supported);
        }

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }
        return No_State_Change;
    }
};


class PE_SNK_Source_Alert_Received_State : public etl::fsm_state<PE, PE_SNK_Source_Alert_Received_State, PE_SNK_Source_Alert_Received, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;
        port.notify_dpm(MsgToDpm_Alert(port.rx_emsg.read32(0)));
        return PE_SNK_Ready;
    }
};


class PE_SNK_Send_EPR_Mode_Entry_State : public etl::fsm_state<PE, PE_SNK_Send_EPR_Mode_Entry_State, PE_SNK_Send_EPR_Mode_Entry, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        EPRMDO eprmdo{};
        eprmdo.action = EPR_MODE_ACTION::ENTER;
        eprmdo.data = pe.dpm.get_epr_watts();

        port.tx_emsg.clear();
        port.tx_emsg.append32(eprmdo.raw_value);

        pe.send_data_msg(PD_DATA_MSGT::EPR_Mode);
        pe.check_request_progress_enter();

        port.timers.start(PD_TIMEOUT::tEnterEPR);

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            return PE_SNK_Ready;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (port.rx_emsg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{port.rx_emsg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_ACKNOWLEDGED) {
                    return PE_SNK_EPR_Mode_Entry_Wait_For_Response;
                }

                port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);
                port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                PE_LOGE("EPR mode entry failed [code 0x{:02x}]", eprmdo.action);

                port.notify_dpm(MsgToDpm_EPREntryFailed(eprmdo.raw_value));
                port.notify_dpm(MsgToDpm_HandshakeDone());

                return PE_SNK_Ready;
            }

            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tSenderResponse) ||
            port.timers.is_expired(PD_TIMEOUT::tEnterEPR))
        {
            return PE_SNK_Send_Soft_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        pe.check_request_progress_exit();

        // On protocol fuckup free `tEnterEPR` timer. In other case
        // it will continue in PE_SNK_EPR_Mode_Entry_Wait_For_Response
        if (port.pe_flags.test(PE_FLAG::PROTOCOL_ERROR)) {
            port.timers.stop(PD_TIMEOUT::tEnterEPR);
        }
    }
};


class PE_SNK_EPR_Mode_Entry_Wait_For_Response_State : public etl::fsm_state<PE, PE_SNK_EPR_Mode_Entry_Wait_For_Response_State, PE_SNK_EPR_Mode_Entry_Wait_For_Response, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_ENTER_STATE_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            if (port.rx_emsg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{port.rx_emsg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_SUCCEEDED) {
                    port.pe_flags.set(PE_FLAG::IN_EPR_MODE);
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                    return PE_SNK_Wait_for_Capabilities;
                }

                PE_LOGE("EPR mode entry failed [code 0x{:02x}]", eprmdo.action);
            }

            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tEnterEPR)) {
            return PE_SNK_Send_Soft_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().port;
        port.timers.stop(PD_TIMEOUT::tEnterEPR);
    }
};


class PE_SNK_EPR_Mode_Exit_Received_State : public etl::fsm_state<PE, PE_SNK_EPR_Mode_Exit_Received_State, PE_SNK_EPR_Mode_Exit_Received, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        if (!pe.is_in_spr_contract()) {
            PE_LOGE("Not in an SPR contract before EPR mode exit => Hard Reset");
            return PE_SNK_Hard_Reset;
        }

        port.pe_flags.clear(PE_FLAG::IN_EPR_MODE);
        port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);

        return PE_SNK_Wait_for_Capabilities;
    }
};


class PE_BIST_Activate_State : public etl::fsm_state<PE, PE_BIST_Activate_State, PE_BIST_Activate, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        // Can enter only when connected at vSafe5V
        if (!port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) { return PE_SNK_Ready; }

        // Simplified check - verify PDO index instead of voltage
        RDO_ANY rdo{port.rdo_contracted};
        if (rdo.obj_position != 1) { return PE_SNK_Ready; }

        // Set up supported modes
        BISTDO bdo{port.rx_emsg.read32(0)};
        if (bdo.mode == BIST_MODE::Carrier) {
            pe.tcpc.req_set_bist(TCPC_BIST_MODE::Carrier);
            return No_State_Change;
        }
        if (bdo.mode == BIST_MODE::TestData) {
            pe.tcpc.req_set_bist(TCPC_BIST_MODE::TestData);
            return No_State_Change;
        }

        // Ignore the rest
        return PE_SNK_Ready;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        // Wait for the TCPC call to complete
        if (!pe.tcpc.is_set_bist_done()) { return No_State_Change; }

        // Small cheat to avoid storing state. Parse BISTDO again; it should
        // not be corrupted in such a short time.
        BISTDO bdo{port.rx_emsg.read32(0)};
        if (bdo.mode == BIST_MODE::Carrier) { return PE_BIST_Carrier_Mode; }
        return PE_BIST_Test_Mode;
    }
};

class PE_BIST_Carrier_Mode_State : public etl::fsm_state<PE, PE_BIST_Carrier_Mode_State, PE_BIST_Carrier_Mode, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.port.timers.start(PD_TIMEOUT::tBISTCarrierMode);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto& port = pe.port;

        if (!pe.tcpc.is_set_bist_done()) { return No_State_Change; }

        if (port.timers.is_disabled(PD_TIMEOUT::tBISTCarrierMode)) {
            return PE_SNK_Ready;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tBISTCarrierMode)) {
            pe.tcpc.req_set_bist(TCPC_BIST_MODE::Off);
            port.timers.stop(PD_TIMEOUT::tBISTCarrierMode);
        }

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& port = get_fsm_context().port;
        port.timers.stop(PD_TIMEOUT::tBISTCarrierMode);
    }
};


class PE_BIST_Test_Mode_State : public etl::fsm_state<PE, PE_BIST_Test_Mode_State, PE_BIST_Test_Mode, MsgSysUpdate, MsgTransitTo> {
public:
    ON_ENTER_STATE_DEFAULT; ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;
        // Ignore everything.
        // Exiting test data mode is only possible via a hard reset.
        port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);
        return No_State_Change;
    }
};


class PE_Give_Revision_State : public etl::fsm_state<PE, PE_Give_Revision_State, PE_Give_Revision, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        auto& port = pe.port;
        pe.log_state();

        RMDO rmdo{};
        rmdo.rev_major = 3;
        rmdo.rev_minor = 2;
        rmdo.ver_major = 1;
        rmdo.ver_minor = 1;

        port.tx_emsg.clear();
        port.tx_emsg.append32(rmdo.raw_value);

        pe.send_data_msg(PD_DATA_MSGT::Revision);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& port = get_fsm_context().port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }
        return No_State_Change;
    }
};


class PE_Src_Disabled_State : public etl::fsm_state<PE, PE_Src_Disabled_State, PE_Src_Disabled, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.port.notify_dpm(MsgToDpm_SrcDisabled());
        return No_State_Change;
    }

    auto on_event(const MsgTransitTo& event) -> etl::fsm_state_id_t {
        if (event.state_id == PE_SNK_Hard_Reset) {
            return event.state_id;
        }
        return No_State_Change;
    }
};


etl_ext::fsm_state_pack<
    PE_SNK_Startup_State,
    PE_SNK_Discovery_State,
    PE_SNK_Wait_for_Capabilities_State,
    PE_SNK_Evaluate_Capability_State,
    PE_SNK_Select_Capability_State,
    PE_SNK_Transition_Sink_State,
    PE_SNK_Ready_State,
    PE_SNK_Give_Sink_Cap_State,
    PE_SNK_EPR_Keep_Alive_State,
    PE_SNK_Hard_Reset_State,
    PE_SNK_Transition_to_default_State,
    PE_SNK_Soft_Reset_State,
    PE_SNK_Send_Soft_Reset_State,
    PE_SNK_Send_Not_Supported_State,
    PE_SNK_Source_Alert_Received_State,
    PE_SNK_Send_EPR_Mode_Entry_State,
    PE_SNK_EPR_Mode_Entry_Wait_For_Response_State,
    PE_SNK_EPR_Mode_Exit_Received_State,
    PE_BIST_Activate_State,
    PE_BIST_Carrier_Mode_State,
    PE_BIST_Test_Mode_State,
    PE_Give_Revision_State,
    PE_Src_Disabled_State
> pe_state_list;

PE::PE(Port& port, IDPM& dpm, PRL& prl, ITCPC& tcpc)
    : etl::fsm(0), port{port}, dpm{dpm}, prl{prl}, tcpc{tcpc}, pe_event_listener{*this}
{
    set_states(pe_state_list.states, pe_state_list.size);
};

void PE::log_state() {
    PE_LOGI("PE state => {}", pe_state_to_desc(get_state_id()));
}

void PE::setup() {
    port.msgbus.subscribe(pe_event_listener);
}

void PE::init() {
    reset();
    port.pe_flags.clear_all();
    port.dpm_requests.clear_all();
    start();
    port.timers.stop_range(PD_TIMERS_RANGE::PE);
}

//
// Msg sending helpers
//
void PE::send_data_msg(PD_DATA_MSGT::Type msgt) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    port.notify_prl(MsgToPrl_DataMsgFromPe{msgt});
}

void PE::send_ext_msg(PD_EXT_MSGT::Type msgt) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    port.notify_prl(MsgToPrl_ExtMsgFromPe{msgt});
}

void PE::send_ctrl_msg(PD_CTRL_MSGT::Type msgt) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    port.notify_prl(MsgToPrl_CtlMsgFromPe{msgt});
}

//
// Utilities
//
auto PE::is_epr_mode_available() const -> bool {
    if (!port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) ||
        port.pe_flags.test(PE_FLAG::EPR_AUTO_ENTER_DISABLED) ||
        port.revision < PD_REVISION::REV30)
    {
        return false;
    }

    // This is not needed, but it exists to suppress warnings from code checkers.
    if (port.source_caps.empty()) { return false; }

    const PDO_FIXED first_src_pdo{port.source_caps[0]};
    return first_src_pdo.epr_capable;
}

bool PE::is_in_epr_mode() const {
    return port.pe_flags.test(PE_FLAG::IN_EPR_MODE);
}


auto PE::is_in_spr_contract() const -> bool {
    const RDO_ANY rdo{port.rdo_contracted};
    return port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) &&
        (rdo.obj_position <= MaxPdoObjects_SPR);
}

auto PE::is_in_pps_contract() const -> bool {
    if (!port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) { return false; }

    const RDO_ANY rdo{port.rdo_contracted};

    // This is not needed, but it exists to suppress warnings from code checkers.
    if (rdo.obj_position == 0 || rdo.obj_position > port.source_caps.size()) {
        return false;
    }

    const PDO_SPR_PPS pdo{port.source_caps[rdo.obj_position-1]};

    return pdo.pdo_type == PDO_TYPE::AUGMENTED &&
        pdo.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_PPS;
}

//
// "Virtual" state to handle a [request] + [response] sequence. The main idea is to
// automate starting the response timer and to restore the DPM task when custom error
// processing is used.
//

void PE::check_request_progress_enter() {
    port.timers.stop(PD_TIMEOUT::tSenderResponse);
    port.pe_flags.clear(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
}

void PE::check_request_progress_exit() {
    port.timers.stop(PD_TIMEOUT::tSenderResponse);
}

auto PE::check_request_progress_run() -> PE_REQUEST_PROGRESS {
    if (!port.pe_flags.test(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED)) {
        if (port.pe_flags.test(PE_FLAG::MSG_DISCARDED)) {
            return PE_REQUEST_PROGRESS::DISCARDED;
        }

        if (port.pe_flags.test(PE_FLAG::PROTOCOL_ERROR)) {
            return PE_REQUEST_PROGRESS::FAILED;
        }

        // Wait for GoodCRC
        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            port.pe_flags.set(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
            // NOTE: This timer can be disabled from RCH chunking. But the flag
            // TRANSMIT_REQUEST_SUCCEEDED protect us from re-arming it. If
            // RCH chunker is activated, then chunker timeout will be used
            // instead to generate error.
            port.timers.start(PD_TIMEOUT::tSenderResponse);
            return PE_REQUEST_PROGRESS::FINISHED;
        }
        return PE_REQUEST_PROGRESS::PENDING;
    }
    return PE_REQUEST_PROGRESS::FINISHED;
}

void PE_EventListener::on_receive(const MsgSysUpdate& msg) {
    switch (pe.local_state) {
        case PE::LOCAL_STATE::DISABLED:
            if (!pe.port.is_attached) { break; }

            __fallthrough;
        case PE::LOCAL_STATE::INIT:
            pe.init();
            pe.local_state = PE::LOCAL_STATE::WORKING;

            __fallthrough;
        case PE::LOCAL_STATE::WORKING:
            if (!pe.port.is_attached) {
                pe.local_state = PE::LOCAL_STATE::DISABLED;
                pe.reset();
                break;
            }
            pe.receive(msg);
            break;
    }
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageReceived&) {
    pe.port.pe_flags.set(PE_FLAG::MSG_RECEIVED);
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageSent&) {
    // Any successful sent inside AMS means first message was sent
    if (pe.port.pe_flags.test(PE_FLAG::AMS_ACTIVE)) {
        pe.port.pe_flags.set(PE_FLAG::AMS_FIRST_MSG_SENT);
    }

    pe.port.pe_flags.set(PE_FLAG::TX_COMPLETE);
    pe.port.wakeup();
}

//
// 8.3.3.4 SOP Soft Reset and Protocol Error State Diagrams
//
// NOTE: May need care; the spec is not clear
//
void PE_EventListener::on_receive(const MsgToPe_PrlReportError& msg) {
    auto& port = pe.port;
    auto err = msg.error;

    if (!pe.is_started()) { return; }

    // Always arm this flag, even for not forwarded errors). This allow
    // optional resource free in `on_exit()`, when some are shared between state.
    //
    // Since only 2 target states possible, ensure both
    // clear this flag in `on_enter()`.
    port.pe_flags.set(PE_FLAG::PROTOCOL_ERROR);

    if (port.pe_flags.test(PE_FLAG::FORWARD_PRL_ERROR)) {
        port.wakeup();
        return;
    }

    if (err == PRL_ERROR::RCH_SEND_FAIL ||
        err == PRL_ERROR::TCH_SEND_FAIL)
    {
        pe.receive(MsgTransitTo(PE_SNK_Send_Soft_Reset));
        port.wakeup();
        return;
    }

    if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) &&
        port.pe_flags.test(PE_FLAG::AMS_ACTIVE) &&
        !port.pe_flags.test(PE_FLAG::AMS_FIRST_MSG_SENT))
    {
        // Discard is not possible without an RX msg, but let's check to be sure.
        if (port.pe_flags.test(PE_FLAG::MSG_RECEIVED)) {
            port.pe_flags.set(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
        }
        pe.receive(MsgTransitTo(PE_SNK_Ready));
        port.wakeup();
        return;
    }

    pe.receive(MsgTransitTo(PE_SNK_Send_Soft_Reset));
    port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlReportDiscard&) {
    pe.port.pe_flags.set(PE_FLAG::MSG_DISCARDED);
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlSoftResetFromPartner&) {
    if (!pe.is_started()) { return; }
    pe.receive(MsgTransitTo(PE_SNK_Soft_Reset));
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetFromPartner&) {
    if (!pe.is_started()) { return; }
    pe.receive(MsgTransitTo(PE_SNK_Transition_to_default));
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetSent&) {
    if (!pe.is_started()) { return; }
    pe.port.pe_flags.clear(PE_FLAG::PRL_HARD_RESET_PENDING);
    pe.port.wakeup();
}

void PE_EventListener::on_receive_unknown(__maybe_unused const etl::imessage& msg) {
    PE_LOGE("PE unknown message, ID: {}", msg.get_message_id());
}

} // namespace pd
