#include <etl/algorithm.h>
#include <etl/array.h>

#include "common_macros.h"
#include "dpm.h"
#include "idriver.h"
#include "pd_conf.h"
#include "pe.h"
#include "port.h"
#include "prl.h"
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
    // PE_SNK_Send_EPR_Mode_Exit,
    PE_SNK_EPR_Mode_Exit_Received, // Manual exit not needed, only SRC-forced

    // [rev3.2] 8.3.3.27.1 BIST Carrier Mode State Diagram
    PE_BIST_Carrier_Mode,

    // [rev3.2] 8.3.3.15.2 Give Revision State Diagram
    PE_Give_Revision,

    // 8.3.3.2.7 PE_SRC_Disabled State
    PE_Src_Disabled,

    // Simple DPM requests for partner info
    PE_Dpm_Get_PPS_Status,
    PE_Dpm_Get_Revision,
    PE_Dpm_Get_Source_Info,

    PE_STATE_COUNT
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
            case PE_BIST_Carrier_Mode: return "PE_BIST_Carrier_Mode";
            case PE_Give_Revision: return "PE_Give_Revision";
            case PE_Src_Disabled: return "PE_Src_Disabled";
            case PE_Dpm_Get_PPS_Status: return "PE_Dpm_Get_PPS_Status";
            case PE_Dpm_Get_Revision: return "PE_Dpm_Get_Revision";
            case PE_Dpm_Get_Source_Info: return "PE_Dpm_Get_Source_Info";
            default: return "Unknown PE state";
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
        pe.log_state();

        pe.port.notify_prl(MsgToPrl_SoftResetFromPe{});
        pe.port.pe_flags.clear(PE_FLAG::HAS_EXPLICIT_CONTRACT);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (!pe.prl.is_running()) { return No_State_Change; }
        return PE_SNK_Discovery;
    }
};


class PE_SNK_Discovery_State : public etl::fsm_state<PE, PE_SNK_Discovery_State, PE_SNK_Discovery, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        // For Sink we detect TC attach via CC1/CC2, with debounce. VBUS should
        // be stable at this moment, so no need to wait.
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

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            auto& msg = pe.port.rx_emsg;

            // Spec requires exact match of caps type and current sink mode
            // to accept.

            if (pe.is_in_epr_mode()) {
                if (msg.is_ext_msg(PD_EXT_MSGT::Source_Capabilities_Extended)) {
                    return PE_SNK_Evaluate_Capability;
                }
            } else {
                if (msg.is_data_msg(PD_DATA_MSGT::Source_Capabilities)) {
                    return PE_SNK_Evaluate_Capability;
                }
            }
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tTypeCSinkWaitCap)) {
            pe.port.pe_flags.set(PE_FLAG::HR_BY_CAPS_TIMEOUT);
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
        pe.log_state();

        auto& msg = pe.port.rx_emsg;

        pe.hard_reset_counter = 0;
        pe.prl.revision = static_cast<PD_REVISION::Type>(
            etl::min(static_cast<uint16_t>(PD_REVISION::REV30), msg.header.spec_revision));

        pe.port.source_caps.clear();
        for (int i = 0, pdo_num = msg.data_size() >> 2; i < pdo_num; i++) {
            pe.port.source_caps.push_back(msg.read32(i*4));
        }

        pe.port.pe_flags.set(PE_FLAG::IS_FROM_EVALUATE_CAPABILITY);
        pe.port.notify_dpm(MsgToDpm_SrcCapsReceived());
        return PE_SNK_Select_Capability;
    }
};

//
// This is the main place, where explicit contract is established / changed.
// We come here in this cases:
//
// 1. Initially, after got Source_Capabilities message.
// 2. After upgrading to EPR and EPR_Source_Capabilities message.
// 3. In PPS mode after timeout.
// 4. After DPM requested to change contract.
//
// This state request desired RDO from DPM. send it to source and wait for
// confirmation. If SRC ask to WAIT - go to READY state (it will repeat after
// delay)
//
// After success, if SRC supports EPR and we are NOT in EPR mode => force
// upgrade. This upgrade is not part of PD spec, but for Sink-only device
// it's the good place to keep things simple.
//
class PE_SNK_Select_Capability_State : public etl::fsm_state<PE, PE_SNK_Select_Capability_State, PE_SNK_Select_Capability, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        // By spec, we should request PDO on previous stage. But for sink-only
        // this place looks more convenient, as unified DPM point for all cases.
        // This decision can be changed later, if needed.
        RDO_ANY rdo{pe.dpm.get_request_data_object(pe.port.source_caps)};

        // Minimal check for RDO validity.
        // DPM implementation MUST NOT return invalid data.
        if (rdo.obj_position < 1 || rdo.obj_position > pe.port.source_caps.size()) {
            PE_LOG("DPM requested RDO with malformed index: {}, doing HW reset", rdo.obj_position);
            return PE_SNK_Hard_Reset;
        }

        // Prepare & send request, depending on SPR/EPR mode
        auto& msg = pe.port.tx_emsg;
        msg.clear();

        // remember RDO, to store after success
        pe.rdo_to_request = rdo.raw_value;

        if (pe.is_in_epr_mode()) {
            msg.append32(rdo.raw_value);
            msg.append32(pe.port.source_caps[rdo.obj_position - 1]);
            pe.send_data_msg(PD_DATA_MSGT::EPR_Request);
        } else {
            msg.append32(rdo.raw_value);
            pe.send_data_msg(PD_DATA_MSGT::Request);
        }

        pe.check_request_progress_enter();
        pe.port.timers.stop(PD_TIMEOUT::tSinkRequest);

        // Don't break on error, process errors manually.
        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto send_status = pe.check_request_progress_run();

        // Reproduce AMS interrupt logic.
        // - If this state is standalone request (DPM) - roll back to Ready
        //   state. DPM means explicit contract already exists.
        // - If we came from Evaluate_Capability - AMS interrupted after first
        //   message => do soft reset
        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            if (pe.port.pe_flags.test(PE_FLAG::IS_FROM_EVALUATE_CAPABILITY)) {
                return PE_SNK_Send_Soft_Reset;
            }
            return PE_SNK_Ready;
        }
        if (send_status == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Accept))
            {
                bool is_first_contract = !pe.port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT);

                pe.port.pe_flags.set(PE_FLAG::HAS_EXPLICIT_CONTRACT);
                pe.rdo_contracted = pe.rdo_to_request;

                if (is_first_contract && (pe.is_in_epr_mode() || !pe.is_epr_mode_available())) {
                    // Report handshake complete, if first contract and should not try EPR
                    pe.port.notify_dpm(MsgToDpm_HandshakeDone());
                }

                if (pe.port.dpm_requests.test_and_clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                    pe.port.notify_dpm(MsgToDpm_NewPowerLevel(true));
                }

                pe.port.notify_dpm(MsgToDpm_SelectCapDone());
                return PE_SNK_Transition_Sink;
            }

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Wait))
            {
                if (pe.port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
                    // Pend another retry. Spec requires to init this timer
                    // on PE_SNK_Ready enter, but it looks more convenient here.
                    pe.port.timers.start(PD_TIMEOUT::tSinkRequest);
                    return PE_SNK_Ready;
                }
                return PE_SNK_Wait_for_Capabilities;
            }

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Reject))
            {
                if (pe.port.dpm_requests.test_and_clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                    pe.port.notify_dpm(MsgToDpm_NewPowerLevel(false));
                }

                if (pe.port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
                    return PE_SNK_Ready;
                }
                return PE_SNK_Wait_for_Capabilities;
            }

            // Anything not expected => soft reset
            return PE_SNK_Send_Soft_Reset;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
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
        pe.log_state();

        // There are 2 timeouts, depending on EPR mode. Both uses the same
        // timer. Use proper one for setup, but SPR one for clear/check
        // (because clear/check use the same timer ID).
        if (pe.port.pe_flags.test(PE_FLAG::IN_EPR_MODE)) {
            pe.port.timers.start(PD_TIMEOUT::tPSTransition_EPR);
        } else {
            pe.port.timers.start(PD_TIMEOUT::tPSTransition_SPR);
        }

        // Any PRL error at this stage should cause hard reset.
        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            if (pe.port.rx_emsg.is_ctrl_msg(PD_CTRL_MSGT::PS_RDY)) {
                return PE_SNK_Ready;
            }
            // Anything else - protocol error
            return PE_SNK_Hard_Reset;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tPSTransition_SPR)) {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
        pe.port.timers.stop(PD_TIMEOUT::tPSTransition_SPR);
    }
};


class PE_SNK_Ready_State : public etl::fsm_state<PE, PE_SNK_Ready_State, PE_SNK_Ready, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        // Ensure to clear flags from past send attempt. If sink returned to
        // this state - all starts from scratch.
        pe.port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        pe.port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);
        pe.port.pe_flags.clear(PE_FLAG::AMS_ACTIVE);
        pe.port.pe_flags.clear(PE_FLAG::AMS_FIRST_MSG_SENT);

        if (pe.is_in_epr_mode()) {
            // If we are in EPR mode, rearm timer for EPR Keep Alive request
            pe.port.timers.start(PD_TIMEOUT::tSinkEPRKeepAlive);
        } else {
            // Force enter EPR mode if possible
            if (pe.is_epr_mode_available()) {
                pe.port.dpm_requests.set(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
            }
        }

        if (pe.is_in_pps_contract()) {
            // PPS contract should be refreshed at least every 10s
            // of inactivity. We use 5s for sure.
            pe.port.timers.start(PD_TIMEOUT::tPPSRequest);
        }

        // Ensure to run after state enter, to proceed pending things (DPM requests)
        pe.port.wakeup();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        bool sr_on_unsupported = pe.port.pe_flags.test(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            auto& msg = pe.port.rx_emsg;
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
                    // No VDM support. Reject for 3.0+, and ignore for 2.0
                    if (!pe.is_rev_2_0()) { return PE_SNK_Send_Not_Supported; }
                    break;

                case PD_DATA_MSGT::BIST:
                    return PE_BIST_Carrier_Mode;

                case PD_DATA_MSGT::Alert:
                    return PE_SNK_Source_Alert_Received;

                case PD_DATA_MSGT::EPR_Mode: {
                    // SRC requested to exit EPR mode (should not happen, but
                    // allowed by the spec)
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

                case PD_CTRL_MSGT::GotoMin: // Deprecated as "not supported"
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
                    // Can not be initiated by SRC, but can be reply after
                    // interrupted AMS. So, do nothing, just ignore garbage
                    // to avoid infinite ping-pong.
                    break;

                case PD_CTRL_MSGT::Get_Revision:
                    return PE_Give_Revision;

                default:
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }
            }
        }

        if (pe.prl.is_busy()) { return No_State_Change; }

        // Special case, process postponed src caps request. If pending - don't
        // try DPM requests queue.
        if (!pe.port.timers.is_disabled(PD_TIMEOUT::tSinkRequest))
        {
            if (pe.port.timers.is_expired(PD_TIMEOUT::tSinkRequest)) {
                pe.port.timers.stop(PD_TIMEOUT::tSinkRequest);
                return PE_SNK_Select_Capability;
            }
        }
        else
        {
            //
            // Process DPM requests
            //
            // Note, request flags are cleared from inside of states, when
            // result determined (succees or failure). Any interrupt of process
            // leaves request armed. Should be ok for sink. Can be changed later.
            //

            pe.port.pe_flags.set(PE_FLAG::AMS_ACTIVE);

            if (pe.port.dpm_requests.test(DPM_REQUEST_FLAG::EPR_MODE_ENTRY)) {
                if (pe.is_in_epr_mode()) {
                    PE_LOG("EPR mode entry requested, but already in EPR mode");
                    pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
                } else if (!pe.is_epr_mode_available()) {
                    PE_LOG("EPR mode entry requested, but not allowed");
                    pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
                } else {
                    return PE_SNK_Send_EPR_Mode_Entry;
                }
            }

            if (pe.port.dpm_requests.test(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                return PE_SNK_Select_Capability;
            }

            if (pe.port.dpm_requests.test(DPM_REQUEST_FLAG::GET_PPS_STATUS)) {
                return PE_Dpm_Get_PPS_Status;
            }

            if (pe.port.dpm_requests.test(DPM_REQUEST_FLAG::GET_REVISION)) {
                return PE_Dpm_Get_Revision;
            }

            if (pe.port.dpm_requests.test(DPM_REQUEST_FLAG::GET_SRC_INFO)) {
                return PE_Dpm_Get_Source_Info;
            }

            pe.port.pe_flags.clear(PE_FLAG::AMS_ACTIVE);
        }

        //
        // Keep-alive for EPR mode / PPS contract
        //

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSinkEPRKeepAlive)) {
            return PE_SNK_EPR_Keep_Alive;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tPPSRequest)) {
            return PE_SNK_Select_Capability;
        }

        // If event caused no activity, emit idle to DPM
        pe.port.notify_dpm(MsgToDpm_Idle());

        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.port.timers.stop(PD_TIMEOUT::tSinkEPRKeepAlive);
        pe.port.timers.stop(PD_TIMEOUT::tPPSRequest);
        pe.port.pe_flags.clear(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
    }
};


class PE_SNK_Give_Sink_Cap_State : public etl::fsm_state<PE, PE_SNK_Give_Sink_Cap_State, PE_SNK_Give_Sink_Cap, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        bool is_epr = pe.port.rx_emsg.header.extended;
        auto& msg = pe.port.tx_emsg;
        msg.clear();

        auto caps = pe.dpm.get_sink_pdo_list();

        // Fill data, length depends on request type
        for (int i = 0, max = caps.size(); i < max; i++) {
            auto pdo = caps[i];

            if (!is_epr) {
                // For regular request (non-EPR) only 7 PDOs allowed
                if (i >= 7) { break; }
            }
            // NOTE: Not sure about this. Do we need zero-padded EPR caps?
            if (pdo == 0) { continue; }

            msg.append32(pdo);
        }

        if (!is_epr) {
            pe.send_data_msg(PD_DATA_MSGT::Sink_Capabilities);
        } else {
            pe.send_ext_msg(PD_EXT_MSGT::EPR_Sink_Capabilities);
        }
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
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
        pe.log_state();

        ECDB ecdb{};
        ecdb.type = PD_EXT_CTRL_MSGT::EPR_KeepAlive;

        auto& msg = pe.port.tx_emsg;
        msg.clear();
        msg.append16(ecdb.raw_value);

        pe.send_ext_msg(PD_EXT_MSGT::Extended_Control);
        pe.check_request_progress_enter();

        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            // If message discarded due another activity => connection is ok,
            // and heartbit is not needed. End with success.
            return PE_SNK_Ready;
        }

        if (send_status == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;

            if (msg.is_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_KeepAlive_Ack)) {
                return PE_SNK_Ready;
            }

            PE_LOG("Protocol error: unexpected message received [0x{:04x}]", msg.header.raw_value);
            return PE_SNK_Send_Soft_Reset;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
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
        pe.log_state();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::HR_BY_CAPS_TIMEOUT) &&
            pe.hard_reset_counter > 2 /* nHardResetCount */)
        {
            return PE_Src_Disabled;
        }

        pe.port.pe_flags.set(PE_FLAG::PRL_HARD_RESET_PENDING);
        pe.port.notify_prl(MsgToPrl_HardResetFromPe{});
        pe.hard_reset_counter++;

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test(PE_FLAG::PRL_HARD_RESET_PENDING)) {
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
        pe.log_state();

        pe.port.pe_flags.clear_all();
        pe.port.dpm_requests.clear_all();

        // If need to pend - call `wait_dpm_transit_to_default(true)` in
        // event handler, and `wait_dpm_transit_to_default(false)` to continue.
        pe.port.notify_dpm(MsgToDpm_TransitToDefault());
        pe.port.wakeup();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (!pe.port.pe_flags.test(PE_FLAG::WAIT_DPM_TRANSIT_TO_DEFAULT)) {
            pe.port.notify_prl(MsgToPrl_PEHardResetDone{});
            return PE_SNK_Startup;
        }
        return No_State_Change;
    }
};


// Come here, when received soft reset from SRC
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
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Wait_for_Capabilities;
        }
        if (pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_DISCARDED)) {
            return PE_SNK_Hard_Reset;
        }
        if (pe.port.pe_flags.test_and_clear(PE_FLAG::PROTOCOL_ERROR)) {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    void on_exit_state() override {
        auto& pe = get_fsm_context();
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
    }
};


class PE_SNK_Send_Soft_Reset_State : public etl::fsm_state<PE, PE_SNK_Send_Soft_Reset_State, PE_SNK_Send_Soft_Reset, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        // Cleanup previous operations flags
        pe.port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        pe.port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);
        pe.port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);

        // Setup error handling and initial state
        pe.port.pe_flags.set(PE_FLAG::CAN_SEND_SOFT_RESET);
        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);

        pe.port.notify_prl(MsgToPrl_SoftResetFromPe{});
        pe.check_request_progress_enter();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        // Wait until PRL layer ready
        if (!pe.prl.is_running()) return No_State_Change;

        // Send only once per state enter
        if (pe.port.pe_flags.test_and_clear(PE_FLAG::CAN_SEND_SOFT_RESET)) {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Soft_Reset);
        }

        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::DISCARDED) {
            return PE_SNK_Ready;
        }

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (pe.port.rx_emsg.is_ctrl_msg(PD_CTRL_MSGT::Accept)) {
                return PE_SNK_Wait_for_Capabilities;
            }
        }

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::PROTOCOL_ERROR) ||
            pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse))
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

        // Reply depends on PD revision. For 3.0+, use Not_Supported,
        if (pe.is_rev_2_0()) {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Reject);
        } else {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Not_Supported);
        }

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }
        return No_State_Change;
    }
};


class PE_SNK_Source_Alert_Received_State : public etl::fsm_state<PE, PE_SNK_Source_Alert_Received_State, PE_SNK_Source_Alert_Received, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_EVENT_NOTHING;

    auto on_enter_state() -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        pe.port.notify_dpm(MsgToDpm_Alert(pe.port.rx_emsg.read32(0)));
        return PE_SNK_Ready;
    }
};


class PE_SNK_Send_EPR_Mode_Entry_State : public etl::fsm_state<PE, PE_SNK_Send_EPR_Mode_Entry_State, PE_SNK_Send_EPR_Mode_Entry, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        EPRMDO eprmdo{};
        eprmdo.action = EPR_MODE_ACTION::ENTER;
        eprmdo.data = pe.dpm.get_epr_watts();

        auto& msg = pe.port.tx_emsg;
        msg.clear();
        msg.append32(eprmdo.raw_value);

        pe.send_data_msg(PD_DATA_MSGT::EPR_Mode);
        pe.check_request_progress_enter();

        pe.port.timers.start(PD_TIMEOUT::tEnterEPR);

        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        auto send_status = pe.check_request_progress_run();

        if ((send_status == PE_REQUEST_PROGRESS::FINISHED) &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;
            if (msg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{msg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_ACKNOWLEDGED) {
                    return PE_SNK_EPR_Mode_Entry_Wait_For_Response;
                }

                pe.port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);
                pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                PE_LOG("EPR mode enter failed [code 0x{:02x}]", eprmdo.action);

                pe.port.notify_dpm(MsgToDpm_EPREntryFailed(eprmdo.raw_value));
                pe.port.notify_dpm(MsgToDpm_HandshakeDone());

                return PE_SNK_Ready;
            }

            return PE_SNK_Send_Soft_Reset;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tEnterEPR)) {
            return PE_SNK_Send_Soft_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() {
        auto& pe = get_fsm_context();
        pe.check_request_progress_exit();
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
    }
};


class PE_SNK_EPR_Mode_Entry_Wait_For_Response_State : public etl::fsm_state<PE, PE_SNK_EPR_Mode_Entry_Wait_For_Response_State, PE_SNK_EPR_Mode_Entry_Wait_For_Response, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_ENTER_STATE_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            auto& msg = pe.port.rx_emsg;
            if (msg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{msg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_SUCCEEDED) {
                    pe.port.pe_flags.set(PE_FLAG::IN_EPR_MODE);
                    pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                    return PE_SNK_Wait_for_Capabilities;
                }

                PE_LOG("EPR mode enter failed [code 0x{:02x}]", eprmdo.action);
            }

            return PE_SNK_Send_Soft_Reset;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tEnterEPR)) {
            return PE_SNK_Send_Soft_Reset;
        }
        return No_State_Change;
    }
};


class PE_SNK_EPR_Mode_Exit_Received_State : public etl::fsm_state<PE, PE_SNK_EPR_Mode_Exit_Received_State, PE_SNK_EPR_Mode_Exit_Received, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO; ON_ENTER_STATE_DEFAULT;

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (!pe.is_in_spr_contract()) {
            PE_LOG("Not in SPR contract before EPR mode exit => Hard Reset");
            return PE_SNK_Hard_Reset;
        }

        pe.port.pe_flags.clear(PE_FLAG::IN_EPR_MODE);
        pe.port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);

        return PE_SNK_Wait_for_Capabilities;
    }
};


class PE_BIST_Carrier_Mode_State : public etl::fsm_state<PE, PE_BIST_Carrier_Mode_State, PE_BIST_Carrier_Mode, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        BISTDO bdo{pe.port.rx_emsg.read32(0)};

        if (bdo.mode == BIST_MODE::Carrier) {
            pe.tcpc.req_bist_carrier_enable(true);
            pe.port.timers.start(PD_TIMEOUT::tBISTCarrierMode);
            return No_State_Change;
        }

        if (bdo.mode == BIST_MODE::TestData) {
            return No_State_Change;
        }

        // Ignore everything else
        return PE_SNK_Ready;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.timers.is_expired(PD_TIMEOUT::tBISTCarrierMode)) {
            pe.tcpc.req_bist_carrier_enable(false);
            return PE_SNK_Ready;
        }

        // Ignore everything.
        // For test data mode exit possible only via hard reset.
        pe.port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);

        return No_State_Change;
    }
};


class PE_Give_Revision_State : public etl::fsm_state<PE, PE_Give_Revision_State, PE_Give_Revision, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        RMDO rmdo{};
        rmdo.rev_major = 3;
        rmdo.rev_minor = 2;
        rmdo.ver_major = 1;
        rmdo.ver_minor = 1;

        auto& msg = pe.port.tx_emsg;
        msg.clear();
        msg.append32(rmdo.raw_value);

        pe.send_data_msg(PD_DATA_MSGT::Revision);
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();

        if (pe.port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
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


class PE_Dpm_Get_PPS_Status_State : public etl::fsm_state<PE, PE_Dpm_Get_PPS_Status_State, PE_Dpm_Get_PPS_Status, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.send_ctrl_msg(PD_CTRL_MSGT::Get_PPS_Status);
        pe.check_request_progress_enter();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::FINISHED &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;
            auto result{0}; // 0 means unsupported

            if (msg.is_ext_msg(PD_EXT_MSGT::PPS_Status)) {
                result = msg.read32(0);
            }

            pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::GET_PPS_STATUS);
            pe.port.notify_dpm(MsgToDpm_PPSStatus(result));
            return PE_SNK_Ready;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Send_Soft_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() {
        auto& pe = get_fsm_context();
        pe.check_request_progress_exit();
    }
};


class PE_Dpm_Get_Revision_State : public etl::fsm_state<PE, PE_Dpm_Get_Revision_State, PE_Dpm_Get_Revision, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.send_ctrl_msg(PD_CTRL_MSGT::Get_Revision);
        pe.check_request_progress_enter();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::FINISHED &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;
            auto result{0}; // 0 means unsupported

            if (msg.is_data_msg(PD_DATA_MSGT::Revision)) {
                result = msg.read32(0);
            }

            pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::GET_REVISION);
            pe.port.notify_dpm(MsgToDpm_PartnerRevision(result));
            return PE_SNK_Ready;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Send_Soft_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() {
        auto& pe = get_fsm_context();
        pe.check_request_progress_exit();
    }
};


class PE_Dpm_Get_Source_Info_State : public etl::fsm_state<PE, PE_Dpm_Get_Source_Info_State, PE_Dpm_Get_Source_Info, MsgSysUpdate, MsgTransitTo> {
public:
    ON_UNKNOWN_EVENT_DEFAULT; ON_TRANSIT_TO;

    auto on_enter_state() -> etl::fsm_state_id_t override {
        auto& pe = get_fsm_context();
        pe.log_state();

        pe.send_ctrl_msg(PD_CTRL_MSGT::Get_Source_Info);
        pe.check_request_progress_enter();
        return No_State_Change;
    }

    auto on_event(const MsgSysUpdate&) -> etl::fsm_state_id_t {
        auto& pe = get_fsm_context();
        auto send_status = pe.check_request_progress_run();

        if (send_status == PE_REQUEST_PROGRESS::FINISHED &&
            pe.port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = pe.port.rx_emsg;
            auto result{0}; // 0 means unsupported

            if (msg.is_data_msg(PD_DATA_MSGT::Source_Info)) {
                result = msg.read32(0);
            }

            pe.port.dpm_requests.clear(DPM_REQUEST_FLAG::GET_SRC_INFO);
            pe.port.notify_dpm(MsgToDpm_SourceInfo(result));
            return PE_SNK_Ready;
        }

        if (pe.port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Send_Soft_Reset;
        }

        return No_State_Change;
    }

    void on_exit_state() {
        auto& pe = get_fsm_context();
        pe.check_request_progress_exit();
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
    PE_BIST_Carrier_Mode_State,
    PE_Give_Revision_State,
    PE_Src_Disabled_State,
    PE_Dpm_Get_PPS_Status_State,
    PE_Dpm_Get_Revision_State,
    PE_Dpm_Get_Source_Info_State
> pe_state_list;

PE::PE(Port& port, IDPM& dpm, PRL& prl, ITCPC& tcpc)
    : etl::fsm(0), port{port}, dpm{dpm}, prl{prl}, tcpc{tcpc}, pe_event_listener{*this}
{
    set_states(pe_state_list.states, pe_state_list.size);
};

void PE::log_state() {
    PE_LOG("PE state => {}", pe_state_to_desc(get_state_id()));
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
void PE::send_data_msg(PD_DATA_MSGT::Type msg) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    prl.send_data_msg(msg);
}

void PE::send_ext_msg(PD_EXT_MSGT::Type msg) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    prl.send_ext_msg(msg);
}

void PE::send_ctrl_msg(PD_CTRL_MSGT::Type msg) {
    port.pe_flags.clear(PE_FLAG::TX_COMPLETE);
    prl.send_ctrl_msg(msg);
}

//
// Utilities
//
auto PE::is_rev_2_0() const -> bool {
    return prl.revision < PD_REVISION::REV30;
}

auto PE::is_epr_mode_available() const -> bool {
    if (!port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) ||
        port.pe_flags.test(PE_FLAG::EPR_AUTO_ENTER_DISABLED) ||
        is_rev_2_0())
    {
        return false;
    }

    const PDO_FIXED fisrt_src_pdo{port.source_caps[0]};

    return fisrt_src_pdo.epr_capable;
}

bool PE::is_in_epr_mode() const {
    return port.pe_flags.test(PE_FLAG::IN_EPR_MODE);
}


auto PE::is_in_spr_contract() const -> bool {
    const RDO_ANY rdo{rdo_contracted};
    return port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) && (rdo.obj_position < 8);
}

auto PE::is_in_pps_contract() const -> bool {
    if (!port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) { return false; }

    const RDO_ANY rdo{rdo_contracted};
    const PDO_SPR_PPS pdo{port.source_caps[rdo.obj_position-1]};

    return pdo.pdo_type == PDO_TYPE::AUGMENTED &&
        pdo.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_PPS;
}

//
// "Virtual" state to proceed [request] + [response] sequence. Main idea is to
// automate response timer start and restore DPM task when custom error
// processing used.
//

void PE::check_request_progress_enter() {
    port.timers.stop(PD_TIMEOUT::tSenderResponse);
    port.pe_flags.clear(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
}

void PE::check_request_progress_exit() {
    port.timers.stop(PD_TIMEOUT::tSenderResponse);
}

auto PE::check_request_progress_run() -> PE_REQUEST_PROGRESS::Type {
    if (!port.pe_flags.test(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED)) {
        // This branch used ONLY when custom error processing selected.
        // For standard cases processing happens in error handler.
        if (port.pe_flags.test(PE_FLAG::MSG_DISCARDED)) {
            return PE_REQUEST_PROGRESS::DISCARDED;
        }

        if (port.pe_flags.test(PE_FLAG::PROTOCOL_ERROR)) {
            return PE_REQUEST_PROGRESS::FAILED;
        }

        // Wait for GoodCRC
        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            port.pe_flags.set(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
            port.timers.start(PD_TIMEOUT::tSenderResponse);
            return PE_REQUEST_PROGRESS::FINISHED;
        }
        return PE_REQUEST_PROGRESS::PENDING;
    }
    return PE_REQUEST_PROGRESS::FINISHED;
}

void PE_EventListener::on_receive(const MsgSysUpdate& msg) {
    switch (pe.local_state) {
        case PE::LS_DISABLED:
            if (!pe.port.is_attached) { break; }

            __fallthrough;
        case PE::LS_INIT:
            pe.init();
            pe.local_state = PE::LS_WORKING;

            __fallthrough;
        case PE::LS_WORKING:
            if (!pe.port.is_attached) {
                pe.local_state = PE::LS_DISABLED;
                pe.reset();
                break;
            }
            pe.receive(msg);
            break;
    }
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageReceived& msg) {
    pe.port.pe_flags.set(PE_FLAG::MSG_RECEIVED);
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageSent& msg) {
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
// NOTE: May be need care, spec is not clear
//
void PE_EventListener::on_receive(const MsgToPe_PrlReportError& msg) {
    auto& port = pe.port;
    auto err = msg.error;

    if (port.pe_flags.test(PE_FLAG::FORWARD_PRL_ERROR)) {
        port.pe_flags.set(PE_FLAG::PROTOCOL_ERROR);
        port.wakeup();
        return;
    }

    if (err == PRL_ERROR::RCH_SEND_FAIL ||
        err ==PRL_ERROR::TCH_SEND_FAIL)
    {
        pe.receive(MsgTransitTo(PE_SNK_Send_Soft_Reset));
        port.wakeup();
        return;
    }

    if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) &&
        port.pe_flags.test(PE_FLAG::AMS_ACTIVE) &&
        !port.pe_flags.test(PE_FLAG::AMS_FIRST_MSG_SENT))
    {
        // Discard is not possible without RX msg, but let's check for sure.
        if (port.pe_flags.test(PE_FLAG::MSG_RECEIVED)) {
            port.pe_flags.set(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
        }
        receive(MsgTransitTo(PE_SNK_Ready));
        port.wakeup();
        return;
    }

    receive(MsgTransitTo(PE_SNK_Send_Soft_Reset));
    port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlReportDiscard& msg) {
    pe.port.pe_flags.set(PE_FLAG::MSG_DISCARDED);
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlSoftResetFromPartner& msg) {
    if (!pe.is_started()) { return; }
    pe.receive(MsgTransitTo(PE_SNK_Soft_Reset));
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetFromPartner& msg) {
    if (!pe.is_started()) { return; }
    pe.tcpc.req_bist_carrier_enable(false);
    pe.receive(MsgTransitTo(PE_SNK_Transition_to_default));
    pe.port.wakeup();
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetSent& msg) {
    if (!pe.is_started()) { return; }
    pe.port.pe_flags.clear(PE_FLAG::PRL_HARD_RESET_PENDING);
    pe.port.wakeup();
}

void PE_EventListener::on_receive_unknown(const etl::imessage& msg) {
    PE_LOG("PE unknown message, id: {}", msg.get_message_id());
}

} // namespace pd
