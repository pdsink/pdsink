#include <etl/algorithm.h>
#include <etl/array.h>

#include "dpm.h"
#include "idriver.h"
#include "pd_log.h"
#include "pe.h"
#include "port.h"
#include "utils/dobj_utils.h"

namespace pd {

using afsm::state_id_t;

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

class InterceptorForwardErrors : public afsm::interceptor<PE, InterceptorForwardErrors> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.port.pe_flags.set(PE_FLAG::FORWARD_PRL_ERROR);
        return No_State_Change;
    }

    static auto on_run_state(PE&) -> state_id_t { return No_State_Change; }

    static void on_exit_state(PE& pe) {
        pe.port.pe_flags.clear(PE_FLAG::FORWARD_PRL_ERROR);
    }
};

class InterceptorCheckRequestProgress : public afsm::interceptor<PE, InterceptorCheckRequestProgress> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.port.pe_flags.clear(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (!port.pe_flags.test(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED)) {
            if (port.pe_flags.test(PE_FLAG::MSG_DISCARDED)) {
                pe.request_progress = PE_REQUEST_PROGRESS::DISCARDED;
                return No_State_Change;
            }

            if (port.pe_flags.test(PE_FLAG::PROTOCOL_ERROR)) {
                pe.request_progress = PE_REQUEST_PROGRESS::FAILED;
                return No_State_Change;
            }

            // Wait for GoodCRC
            if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
                port.pe_flags.set(PE_FLAG::TRANSMIT_REQUEST_SUCCEEDED);
                // NOTE: This timer can be disabled from RCH chunking. But the flag
                // TRANSMIT_REQUEST_SUCCEEDED protect us from re-arming it. If
                // RCH chunker is activated, then chunker timeout will be used
                // instead to generate error.
                port.timers.start(PD_TIMEOUT::tSenderResponse);
                pe.request_progress = PE_REQUEST_PROGRESS::FINISHED;
                return No_State_Change;
            }

            pe.request_progress = PE_REQUEST_PROGRESS::PENDING;
            return No_State_Change;
        }

        pe.request_progress = PE_REQUEST_PROGRESS::FINISHED;
        return No_State_Change;
    }

    static void on_exit_state(PE& pe) {
        pe.port.timers.stop(PD_TIMEOUT::tSenderResponse);
    }
};

class PE_SNK_Startup_State : public afsm::state<PE, PE_SNK_Startup_State, PE_SNK_Startup> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        port.notify_prl(MsgToPrl_EnqueueRestart{});
        port.pe_flags.clear(PE_FLAG::HAS_EXPLICIT_CONTRACT);
        port.notify_dpm(MsgToDpm_Startup{});
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (!port.is_prl_running()) {
            PE_LOGD("PRL is not running, wait...");
            return No_State_Change;
        }
        return PE_SNK_Discovery;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_Discovery_State : public afsm::state<PE, PE_SNK_Discovery_State, PE_SNK_Discovery> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.log_state();

        // As a Sink, we detect TC attach via CC1/CC2 with debounce. VBUS should
        // be stable at this moment, so there is no need to wait.
        return PE_SNK_Wait_for_Capabilities;
    }

    static state_id_t on_run_state(PE&) { return No_State_Change; }
    static void on_exit_state(PE&) {}
};


class PE_SNK_Wait_for_Capabilities_State : public afsm::state<PE, PE_SNK_Wait_for_Capabilities_State, PE_SNK_Wait_for_Capabilities> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.log_state();

        pe.port.timers.start(PD_TIMEOUT::tTypeCSinkWaitCap);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            // The spec requires an exact match of the capabilities type and the
            // current sink mode to accept.
            if (pe.is_in_epr_mode()) {
                if (port.rx_emsg.is_ext_msg(PD_EXT_MSGT::EPR_Source_Capabilities)) {
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

    static void on_exit_state(PE& pe) {
        pe.port.timers.stop(PD_TIMEOUT::tTypeCSinkWaitCap);
    }
};


class PE_SNK_Evaluate_Capability_State : public afsm::state<PE, PE_SNK_Evaluate_Capability_State, PE_SNK_Evaluate_Capability> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        port.source_caps.clear();
        for (int i = 0; i < port.rx_emsg.size_to_pdo_count(); i++) {
            port.source_caps.push_back(port.rx_emsg.read32(i*4));
        }

        pe.log_source_caps();

        if (!pe.validate_source_caps(port.source_caps)) {
            PE_LOGE("Source_Capabilities validation failed");
            return PE_SNK_Send_Not_Supported;
        }

        // Continue after all validation checks passed
        port.hard_reset_counter = 0;
        port.revision = static_cast<PD_REVISION::Type>(
            etl::min(static_cast<uint16_t>(MaxSupportedRevision), port.rx_emsg.header.spec_revision));

        if (port.source_caps.size() > MaxPdoObjects_SPR && !pe.is_in_epr_mode()) {
            // NOTE: For unknown reasons, spec does NOT say EPR_Source_Capabilities
            // is the fuckup in SPR mode, when received in Ready state.
            // So, process it as valid, but cut the size.
            PE_LOGE("Source sent too many PDOs for SPR mode ({}), cutting to {}",
                port.source_caps.size(), MaxPdoObjects_SPR);
            port.source_caps.resize(MaxPdoObjects_SPR);
        }

        port.notify_dpm(MsgToDpm_SrcCapsReceived());
        return PE_SNK_Select_Capability;
    }

    static state_id_t on_run_state(PE&) { return No_State_Change; }
    static void on_exit_state(PE&) {}
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

class PE_SNK_Select_Capability_State :
    public afsm::state<PE, PE_SNK_Select_Capability_State, PE_SNK_Select_Capability>,
    public afsm::interceptor_pack<InterceptorCheckRequestProgress, InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        auto rdo_and_pdo = pe.dpm.get_request_data_object(port.source_caps);

        PE_LOGD("Selecting PDO[{}] (counting from 1), RDO is 0x{:08X}",
            RDO_ANY{rdo_and_pdo.first}.obj_position, rdo_and_pdo.first);

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

        // Cleanup postponed retry if existed
        port.timers.stop(PD_TIMEOUT::tSinkRequest);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        // Reproduce AMS interrupt logic:
        // - If this state is a standalone request (DPM), roll back to the Ready
        //   state. DPM means an explicit contract already exists.
        // - If we came from Evaluate_Capability and the AMS was interrupted after
        //   the first message => perform a Soft Reset.
        if (pe.request_progress == PE_REQUEST_PROGRESS::DISCARDED) {
            if (pe.get_previous_state_id() == PE_SNK_Evaluate_Capability) {
                return PE_SNK_Send_Soft_Reset;
            }
            return PE_SNK_Ready;
        }
        if (pe.request_progress == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((pe.request_progress == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            auto& msg = port.rx_emsg;

            if (msg.is_ctrl_msg(PD_CTRL_MSGT::Accept))
            {
                port.pe_flags.set(PE_FLAG::HAS_EXPLICIT_CONTRACT);
                port.rdo_contracted = port.rdo_to_request;

                if (pe.active_dpm_request == DPM_REQUEST_FLAG::NEW_POWER_LEVEL) {
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL);
                    port.notify_dpm(MsgToDpm_NewPowerLevelAccepted{});
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
                if (pe.active_dpm_request == DPM_REQUEST_FLAG::NEW_POWER_LEVEL) {
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::NEW_POWER_LEVEL);
                    port.notify_dpm(MsgToDpm_NewPowerLevelRejected{});
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

    static void on_exit_state(PE&) {}
};


class PE_SNK_Transition_Sink_State :
    public afsm::state<PE, PE_SNK_Transition_Sink_State, PE_SNK_Transition_Sink>,
    // Any PRL error at this stage should cause a hard reset.
    public afsm::interceptor_pack<InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            if (port.rx_emsg.is_ctrl_msg(PD_CTRL_MSGT::PS_RDY)) {
                port.notify_dpm(MsgToDpm_SnkReady{});

                if (!port.pe_flags.test(PE_FLAG::HANDSHAKE_REPORTED) &&
                    (pe.is_in_epr_mode() || !pe.is_epr_mode_available())) {
                    // Report handshake complete if not done before, and we
                    // should not try to enter EPR (already there or not supported)
                    port.pe_flags.set(PE_FLAG::HANDSHAKE_REPORTED);
                    port.notify_dpm(MsgToDpm_HandshakeDone());
                }
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

    static void on_exit_state(PE& pe) {
        auto& port = pe.port;
        port.timers.stop(PD_TIMEOUT::tPSTransition_SPR);
    }
};


class PE_SNK_Ready_State : public afsm::state<PE, PE_SNK_Ready_State, PE_SNK_Ready> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (pe.get_previous_state_id() == PE_SNK_EPR_Keep_Alive) {
            // Log returning from EPR Keep-Alive at debug level to reduce noise
            PE_LOGV("PE state => {}", pe_state_to_desc(pe.get_state_id()));
        } else {
            pe.log_state();
        }

        // Ensure flags from the previous send attempt are cleared.
        // If the sink returned to this state, everything starts from scratch.
        port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);
        port.pe_flags.clear(PE_FLAG::AMS_ACTIVE);
        port.pe_flags.clear(PE_FLAG::AMS_FIRST_MSG_SENT);

        pe.active_dpm_request = DPM_REQUEST_FLAG::NONE;

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
        pe.request_wakeup();
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
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
                case PD_EXT_MSGT::EPR_Source_Capabilities:
                    if (!pe.is_in_epr_mode()) {
                        // NOTE: This case is NOT specified explicitly in the
                        // spec. Just log but cut the size in the evaluation
                        // state.
                        PE_LOGE("Got EPR_Source_Capabilities in SPR mode. Will take only SPR part.");
                    }
                    return PE_SNK_Evaluate_Capability;

                case PD_EXT_MSGT::Extended_Control:
                    if (msg.is_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Get_Sink_Cap)) {
                        return PE_SNK_Give_Sink_Cap;
                    }
                    {
                        ETL_MAYBE_UNUSED ECDB ecdb{msg.read16(0)};
                        PE_LOGE("Unsupported PD_EXT_MSGT::Extended_Control type: {}", ecdb.type);
                    }
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;

                default:
                    PE_LOGE("Unexpected PD_EXT_MSGT: {}", hdr.message_type);
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
                    if (pe.is_in_epr_mode()) {
                        PE_LOGE("Got SPR Source_Capabilities in EPR mode => Hard Reset");
                        return PE_SNK_Hard_Reset;
                    }
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
                    PE_LOGE("Unsupported PD_DATA_MSGT::EPR_Mode Action: {}", eprmdo.action);
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }

                default:
                    PE_LOGE("Unexpected PD_DATA_MSGT: {}", hdr.message_type);
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
                    PE_LOGE("Unexpected Accept/Reject => Soft Reset");
                    return PE_SNK_Send_Soft_Reset;

                case PD_CTRL_MSGT::Ping: // Deprecated as "ignored"
                    break;

                case PD_CTRL_MSGT::PS_RDY: // Unexpected => soft reset
                    PE_LOGE("Unexpected PD_CTRL_MSGT::PS_RDY => Soft Reset");
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
                    PE_LOGE("Unexpected PD_CTRL_MSGT: {}", hdr.message_type);
                    return sr_on_unsupported ? PE_SNK_Send_Soft_Reset : PE_SNK_Send_Not_Supported;
                }
            }
        }

        if (port.is_prl_busy()) {
            PE_LOGD("PRL is busy, wait...");
            return No_State_Change;
        }

        // Special case: SRC postponed Request (Select Capability) via `Wait`.
        // pause and don't try to bomb SRC with pending DPM requests.
        if (!port.timers.is_disabled(PD_TIMEOUT::tSinkRequest))
        {
            if (port.timers.is_expired(PD_TIMEOUT::tSinkRequest)) {
                port.timers.stop(PD_TIMEOUT::tSinkRequest);

                // Note, if postponed Request was initialized by DPM, such
                // simplified state change will cause duplicated command.
                // But since `Wait` scenario is very rare, that's acceptable.
                // Let's keep things simple for now.
                return PE_SNK_Select_Capability;
            }
        }

        // If SRC requested Wait before, we should not process DPM requests
        // until timeout compleete.
        if (port.timers.is_disabled(PD_TIMEOUT::tSinkRequest))
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
                    pe.active_dpm_request = DPM_REQUEST_FLAG::EPR_MODE_ENTRY;
                    return PE_SNK_Send_EPR_Mode_Entry;
                }
            }

            if (port.dpm_requests.test(DPM_REQUEST_FLAG::NEW_POWER_LEVEL)) {
                pe.active_dpm_request = DPM_REQUEST_FLAG::NEW_POWER_LEVEL;
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

        return No_State_Change;
    }

    static void on_exit_state(PE& pe) {
        auto& port = pe.port;
        port.timers.stop(PD_TIMEOUT::tSinkEPRKeepAlive);
        port.timers.stop(PD_TIMEOUT::tPPSRequest);
        port.pe_flags.clear(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
    }
};


class PE_SNK_Give_Sink_Cap_State : public afsm::state<PE, PE_SNK_Give_Sink_Cap_State, PE_SNK_Give_Sink_Cap> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }

        // No more checks - rely on standard error processing.
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_EPR_Keep_Alive_State :
    public afsm::state<PE, PE_SNK_EPR_Keep_Alive_State, PE_SNK_EPR_Keep_Alive>,
    public afsm::interceptor_pack<InterceptorCheckRequestProgress, InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        // Manually log as debug level, to reduce noise
        PE_LOGV("PE state => {}", pe_state_to_desc(pe.get_state_id()));


        ECDB ecdb{};
        ecdb.type = PD_EXT_CTRL_MSGT::EPR_KeepAlive;

        port.tx_emsg.clear();
        port.tx_emsg.append16(ecdb.raw_value);

        pe.send_ext_msg(PD_EXT_MSGT::Extended_Control);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (pe.request_progress == PE_REQUEST_PROGRESS::DISCARDED) {
            // If the message was discarded due to another activity => the connection
            // is OK, and a heartbeat is not needed. Consider it successful.
            return PE_SNK_Ready;
        }

        if (pe.request_progress == PE_REQUEST_PROGRESS::FAILED) {
            return PE_SNK_Send_Soft_Reset;
        }

        if ((pe.request_progress == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (port.rx_emsg.is_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_KeepAlive_Ack)) {
                return PE_SNK_Ready;
            }

            PE_LOGE("Protocol error: unexpected message received [0x{:08X}]", port.rx_emsg.header.raw_value);
            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tSenderResponse)) {
            return PE_SNK_Hard_Reset;
        }

        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_Hard_Reset_State : public afsm::state<PE, PE_SNK_Hard_Reset_State, PE_SNK_Hard_Reset> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

    static auto on_run_state(PE& pe) -> state_id_t {
        if (pe.port.pe_flags.test(PE_FLAG::PRL_HARD_RESET_PENDING)) {
            return No_State_Change;
        }
        return PE_SNK_Transition_to_default;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_Transition_to_default_State : public afsm::state<PE, PE_SNK_Transition_to_default_State, PE_SNK_Transition_to_default> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        port.pe_flags.clear_all();
        port.dpm_requests.clear_all();

        // If you need to pend, call `wait_dpm_transit_to_default(true)` in the
        // event handler, and `wait_dpm_transit_to_default(false)` to continue.
        port.notify_dpm(MsgToDpm_TransitToDefault());
        pe.request_wakeup();
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (!port.pe_flags.test(PE_FLAG::WAIT_DPM_TRANSIT_TO_DEFAULT)) {
            port.notify_prl(MsgToPrl_PEHardResetDone{});
            return PE_SNK_Startup;
        }
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


// Come here when a Soft Reset is received from the SRC
class PE_SNK_Soft_Reset_State :
    public afsm::state<PE, PE_SNK_Soft_Reset_State, PE_SNK_Soft_Reset>,
    public afsm::interceptor_pack<InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.log_state();

        pe.send_ctrl_msg(PD_CTRL_MSGT::Accept);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Wait_for_Capabilities;
        }
        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_DISCARDED) ||
            port.pe_flags.test_and_clear(PE_FLAG::PROTOCOL_ERROR)) {
            return PE_SNK_Hard_Reset;
        }
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_Send_Soft_Reset_State :
    public afsm::state<PE, PE_SNK_Send_Soft_Reset_State, PE_SNK_Send_Soft_Reset>,
    public afsm::interceptor_pack<InterceptorCheckRequestProgress, InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        // Clean up flags from previous operations
        port.pe_flags.clear(PE_FLAG::MSG_DISCARDED);
        port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);
        port.pe_flags.clear(PE_FLAG::PROTOCOL_ERROR);

        port.pe_flags.set(PE_FLAG::CAN_SEND_SOFT_RESET);

        port.notify_prl(MsgToPrl_EnqueueRestart{});
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        // Wait until the PRL layer is ready
        if (!port.is_prl_running()) {
            PE_LOGD("PRL is not running, wait...");
            return No_State_Change;
        }

        // Send only once per state entry
        if (port.pe_flags.test_and_clear(PE_FLAG::CAN_SEND_SOFT_RESET)) {
            pe.send_ctrl_msg(PD_CTRL_MSGT::Soft_Reset);
            return No_State_Change;
        }

        // NOTE: here was the right place for status check before using
        // interceptors.

        if (pe.request_progress == PE_REQUEST_PROGRESS::DISCARDED) {
            return PE_SNK_Ready;
        }

        if ((pe.request_progress == PE_REQUEST_PROGRESS::FINISHED) &&
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

    static void on_exit_state(PE&) {}
};


class PE_SNK_Send_Not_Supported_State : public afsm::state<PE, PE_SNK_Send_Not_Supported_State, PE_SNK_Send_Not_Supported> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_SNK_Source_Alert_Received_State : public afsm::state<PE, PE_SNK_Source_Alert_Received_State, PE_SNK_Source_Alert_Received> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        port.notify_dpm(MsgToDpm_Alert(port.rx_emsg.read32(0)));
        return PE_SNK_Ready;
    }

    static state_id_t on_run_state(PE&) { return No_State_Change; }
    static void on_exit_state(PE&) {}
};


class PE_SNK_Send_EPR_Mode_Entry_State :
    public afsm::state<PE, PE_SNK_Send_EPR_Mode_Entry_State, PE_SNK_Send_EPR_Mode_Entry>,
    public afsm::interceptor_pack<InterceptorCheckRequestProgress>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;
        pe.log_state();

        EPRMDO eprmdo{};
        eprmdo.action = EPR_MODE_ACTION::ENTER;
        eprmdo.data = pe.dpm.get_epr_watts();

        port.tx_emsg.clear();
        port.tx_emsg.append32(eprmdo.raw_value);

        pe.send_data_msg(PD_DATA_MSGT::EPR_Mode);

        port.timers.start(PD_TIMEOUT::tEnterEPR);

        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (pe.request_progress == PE_REQUEST_PROGRESS::DISCARDED) {
            return PE_SNK_Ready;
        }

        if ((pe.request_progress == PE_REQUEST_PROGRESS::FINISHED) &&
            port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED))
        {
            if (port.rx_emsg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{port.rx_emsg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_ACKNOWLEDGED) {
                    return PE_SNK_EPR_Mode_Entry_Wait_For_Response;
                }

                port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);
                port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                PE_LOGE("EPR mode entry failed [code 0x{:02X}]", eprmdo.action);
                port.notify_dpm(MsgToDpm_EPREntryFailed(eprmdo.raw_value));

                if (!port.pe_flags.test(PE_FLAG::HANDSHAKE_REPORTED)) {
                    port.pe_flags.set(PE_FLAG::HANDSHAKE_REPORTED);
                    port.notify_dpm(MsgToDpm_HandshakeDone());
                }

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

    static void on_exit_state(PE& pe) {
        auto& port = pe.port;

        // On protocol fuckup free `tEnterEPR` timer. In other case
        // it will continue in PE_SNK_EPR_Mode_Entry_Wait_For_Response
        if (port.pe_flags.test(PE_FLAG::PROTOCOL_ERROR)) {
            port.timers.stop(PD_TIMEOUT::tEnterEPR);
        }
    }
};


class PE_SNK_EPR_Mode_Entry_Wait_For_Response_State : public afsm::state<PE, PE_SNK_EPR_Mode_Entry_Wait_For_Response_State, PE_SNK_EPR_Mode_Entry_Wait_For_Response> {
public:
    static state_id_t on_enter_state(PE& pe) {
        pe.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (port.pe_flags.test_and_clear(PE_FLAG::MSG_RECEIVED)) {
            if (port.rx_emsg.is_data_msg(PD_DATA_MSGT::EPR_Mode)) {
                EPRMDO eprmdo{port.rx_emsg.read32(0)};

                if (eprmdo.action == EPR_MODE_ACTION::ENTER_SUCCEEDED) {
                    port.pe_flags.set(PE_FLAG::IN_EPR_MODE);
                    port.dpm_requests.clear(DPM_REQUEST_FLAG::EPR_MODE_ENTRY);

                    return PE_SNK_Wait_for_Capabilities;
                }

                PE_LOGE("EPR mode entry failed [code 0x{:02X}]", eprmdo.action);
            }

            return PE_SNK_Send_Soft_Reset;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tEnterEPR)) {
            return PE_SNK_Send_Soft_Reset;
        }
        return No_State_Change;
    }

    static void on_exit_state(PE& pe) {
        pe.port.timers.stop(PD_TIMEOUT::tEnterEPR);
    }
};


class PE_SNK_EPR_Mode_Exit_Received_State : public afsm::state<PE, PE_SNK_EPR_Mode_Exit_Received_State, PE_SNK_EPR_Mode_Exit_Received> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (!pe.is_in_spr_contract()) {
            PE_LOGE("Not in an SPR contract before EPR mode exit => Hard Reset");
            return PE_SNK_Hard_Reset;
        }

        port.pe_flags.clear(PE_FLAG::IN_EPR_MODE);
        port.pe_flags.set(PE_FLAG::EPR_AUTO_ENTER_DISABLED);

        return PE_SNK_Wait_for_Capabilities;
    }

    static state_id_t on_run_state(PE&) { return No_State_Change; }
    static void on_exit_state(PE&) {}
};


class PE_BIST_Activate_State : public afsm::state<PE, PE_BIST_Activate_State, PE_BIST_Activate> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        // Wait for the TCPC call to complete
        if (!pe.tcpc.is_set_bist_done()) { return No_State_Change; }

        // Small cheat to avoid storing state. Parse BISTDO again; it should
        // not be corrupted in such a short time.
        BISTDO bdo{port.rx_emsg.read32(0)};
        if (bdo.mode == BIST_MODE::Carrier) { return PE_BIST_Carrier_Mode; }
        return PE_BIST_Test_Mode;
    }

    static void on_exit_state(PE&) {}
};

class PE_BIST_Carrier_Mode_State : public afsm::state<PE, PE_BIST_Carrier_Mode_State, PE_BIST_Carrier_Mode> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.log_state();

        pe.port.timers.start(PD_TIMEOUT::tBISTCarrierMode);
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        auto& port = pe.port;

        if (!pe.tcpc.is_set_bist_done()) { return No_State_Change; }

        if (port.timers.is_disabled(PD_TIMEOUT::tBISTCarrierMode)) {
            return PE_SNK_Transition_to_default;
        }

        if (port.timers.is_expired(PD_TIMEOUT::tBISTCarrierMode)) {
            pe.tcpc.req_set_bist(TCPC_BIST_MODE::Off);
            port.timers.stop(PD_TIMEOUT::tBISTCarrierMode);
        }

        return No_State_Change;
    }

    static void on_exit_state(PE& pe) {
        pe.port.timers.stop(PD_TIMEOUT::tBISTCarrierMode);
    }
};


class PE_BIST_Test_Mode_State : public afsm::state<PE, PE_BIST_Test_Mode_State, PE_BIST_Test_Mode> {
public:
    static state_id_t on_enter_state(PE& pe) {
        pe.log_state();
        return No_State_Change;
    }

    static auto on_run_state(PE& pe) -> state_id_t {
        // Ignore everything.
        // Exiting test data mode is only possible via a hard reset.
        pe.port.pe_flags.clear(PE_FLAG::MSG_RECEIVED);
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_Give_Revision_State : public afsm::state<PE, PE_Give_Revision_State, PE_Give_Revision> {
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
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

    static auto on_run_state(PE& pe) -> state_id_t {
        if (pe.port.pe_flags.test_and_clear(PE_FLAG::TX_COMPLETE)) {
            return PE_SNK_Ready;
        }
        return No_State_Change;
    }

    static void on_exit_state(PE&) {}
};


class PE_Src_Disabled_State :
    public afsm::state<PE, PE_Src_Disabled_State, PE_Src_Disabled>,
    // Don't leave state on error. Only allow exit via Hard Reset from
    // partner, if cable stays connected.
    public afsm::interceptor_pack<InterceptorForwardErrors>
{
public:
    static auto on_enter_state(PE& pe) -> state_id_t {
        pe.log_state();

        pe.port.notify_dpm(MsgToDpm_SrcDisabled());
        return No_State_Change;
    }

    static state_id_t on_run_state(PE&) { return No_State_Change; }

    static void on_exit_state(PE&) {}
};


using PE_STATES = afsm::state_pack<
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
>;

PE::PE(Port& port, IDPM& dpm, PRL& prl, ITCPC& tcpc)
    : port{port}, dpm{dpm}, prl{prl}, tcpc{tcpc}, pe_event_listener{*this}
{
    set_states<PE_STATES>();
}

void PE::log_state() const {
    PE_LOGI("PE state => {}", pe_state_to_desc(get_state_id()));
}

void PE::log_source_caps() const {
    using namespace dobj_utils;

    int caps_count = port.source_caps.size();
    PE_LOGI("Total source capabilities: {}", caps_count);

    for (int i = 0; i < caps_count; i++) {
        auto pdo = port.source_caps[i];

        if (pdo == 0) {
            PE_LOGI("  PDO[{}]: <PLACEHOLDER> (zero)", i+1);
            continue;
        }

        auto id = get_src_pdo_variant(pdo);

        if (id == PDO_VARIANT::UNKNOWN) {
            PE_LOGI("  PDO[{}]: 0x{:08X} <UNKNOWN>", i+1, pdo);
        }
        else if (id == PDO_VARIANT::FIXED) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            PE_LOGI("  PDO[{}]: 0x{:08X} <FIXED> {}mV {}mA",
                i+1, pdo, limits.mv_min, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_PPS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            PE_LOGI("  PDO[{}]: 0x{:08X} <APDO_PPS> {}-{}mV {}mA",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_SPR_AVS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            PE_LOGI("  PDO[{}]: 0x{:08X} <APDO_SPR_AVS> {}-{}mV {}mA",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_EPR_AVS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            PE_LOGI("  PDO[{}]: 0x{:08X} <APDO_EPR_AVS> {}-{}mV {}W",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.pdp);
        }
        else {
            PE_LOGI("  PDO[{}]: 0x{:08X} <!!!UNHANDLED!!!>", i+1, pdo);
        }
    }
}

void PE::setup() {
    port.pe_rtr = &pe_event_listener;
}

void PE::init() {
    change_state(afsm::Uninitialized);
    port.pe_flags.clear_all();
    port.dpm_requests.clear_all();
    port.revision = MaxSupportedRevision;
    active_dpm_request = DPM_REQUEST_FLAG::NONE;
    port.timers.stop_range(PD_TIMERS_RANGE::PE);
    change_state(PE_SNK_Startup);
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

bool PE::validate_source_caps(const etl::ivector<uint32_t>& src_caps) {
    using namespace dobj_utils;

    if (src_caps.empty()) {
        PE_LOGE("SRC Capabilities can't be empty");
        return false;
    }

    if (src_caps.size() > MaxPdoObjects) {
        PE_LOGE("SRC Capabilities max count is {}, got {}", MaxPdoObjects, src_caps.size());
        return false;
    }

    // First PDO must be Safe5v
    if (get_src_pdo_variant(src_caps[0]) != PDO_VARIANT::FIXED ||
        PDO_FIXED{src_caps[0]}.voltage != 100 /* 5000mV in 50mv steps */)
    {
        PE_LOGE("First PDO MUST be Safe5v FIXED");
        return false;
    }

    // EPR PDOs are prohibited at SPR positions (1-7, counted from 1),
    // and SPR PDOs are prohibited at EPR positions (8+).
    for (int i = 0; i < etl::min<int>(src_caps.size(), MaxPdoObjects); i++) {
        auto pdo = src_caps[i];
        auto pdo_variant = get_src_pdo_variant(pdo);
        if (pdo_variant == PDO_VARIANT::APDO_EPR_AVS ||
            (pdo_variant == PDO_VARIANT::FIXED && PDO_FIXED{pdo}.voltage > 400 /* >20V  in 50mv steps */))
        {
            if (i < MaxPdoObjects_SPR) {
                PE_LOGE("EPR PDO prohibited at SPR position {}", i + 1);
                return false;
            }
        } else {
            if (i >= MaxPdoObjects_SPR) {
                PE_LOGE("SPR PDO prohibited at EPR position {}", i + 1);
                return false;
            }
        }
    }

    // Count AVS APDOs: max 1 SPR AVS and max 1 EPR AVS
    int spr_avs_count = 0;
    int epr_avs_count = 0;

    for (size_t i = 0; i < src_caps.size(); i++) {
        auto pdo_variant = get_src_pdo_variant(src_caps[i]);
        if (pdo_variant == PDO_VARIANT::APDO_SPR_AVS) {
            spr_avs_count++;
            if (spr_avs_count > 1) {
                PE_LOGE("Only one SPR AVS APDO allowed");
                return false;
            }
        } else if (pdo_variant == PDO_VARIANT::APDO_EPR_AVS) {
            epr_avs_count++;
            if (epr_avs_count > 1) {
                PE_LOGE("Only one EPR AVS APDO allowed");
                return false;
            }
        }
    }

    // Check Fixed PDO voltages are strictly ascending (no duplicates)
    uint32_t prev_fixed_voltage = 0;
    for (size_t i = 0; i < src_caps.size(); i++) {
        auto pdo_variant = get_src_pdo_variant(src_caps[i]);
        if (pdo_variant == PDO_VARIANT::FIXED) {
            uint32_t voltage = PDO_FIXED{src_caps[i]}.voltage;
            if (voltage <= prev_fixed_voltage) {
                PE_LOGE("Fixed PDO voltages must be strictly ascending");
                return false;
            }
            prev_fixed_voltage = voltage;
        }
    }

    // Check PPS APDO max_voltage is ascending (not strictly - duplicates allowed)
    uint32_t prev_pps_max_voltage = 0;
    for (size_t i = 0; i < src_caps.size(); i++) {
        auto pdo_variant = get_src_pdo_variant(src_caps[i]);
        if (pdo_variant == PDO_VARIANT::APDO_PPS) {
            uint32_t max_voltage = PDO_SPR_PPS{src_caps[i]}.max_voltage;
            if (max_voltage < prev_pps_max_voltage) {
                PE_LOGE("PPS APDO max_voltage must be in ascending order");
                return false;
            }
            prev_pps_max_voltage = max_voltage;
        }
    }

    return true;
}

void PE_EventListener::on_receive(const MsgSysUpdate&) {
    switch (pe.local_state) {
        case PE::LOCAL_STATE::DISABLED:
            if (!pe.port.is_attached) { break; }

            ETL_FALLTHROUGH;
        case PE::LOCAL_STATE::INIT:
            pe.init();
            pe.local_state = PE::LOCAL_STATE::WORKING;

            ETL_FALLTHROUGH;
        case PE::LOCAL_STATE::WORKING:
            if (!pe.port.is_attached) {
                pe.local_state = PE::LOCAL_STATE::DISABLED;
                pe.change_state(afsm::Uninitialized);
                break;
            }
            pe.run();
            break;
    }

    if (pe.has_deferred_wakeup_request) {
        pe.has_deferred_wakeup_request = false;
        pe.request_wakeup();
    }
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageReceived&) {
    PE_LOGV("Message received (PRL notification to PE)");

    pe.port.pe_flags.set(PE_FLAG::MSG_RECEIVED);
}

void PE_EventListener::on_receive(const MsgToPe_PrlMessageSent&) {
    PE_LOGV("Message transferred (PRL notification to PE)");

    // Any successful send inside AMS means the first message was sent
    if (pe.port.pe_flags.test(PE_FLAG::AMS_ACTIVE)) {
        pe.port.pe_flags.set(PE_FLAG::AMS_FIRST_MSG_SENT);
    }

    pe.port.pe_flags.set(PE_FLAG::TX_COMPLETE);
}

//
// 8.3.3.4 SOP Soft Reset and Protocol Error State Diagrams
//
// NOTE: Needs attention; the specification is unclear here
//
void PE_EventListener::on_receive(const MsgToPe_PrlReportError& msg) {
    auto& port = pe.port;
    auto err = msg.error;

    if (pe.is_uninitialized()) { return; }

    // Always arm this flag, even for not forwarded errors). This allow
    // optional resource free in `on_exit()`, when some are shared between state.
    //
    // Since only 2 target states possible, ensure both
    // clear this flag in `on_exit()`.
    port.pe_flags.set(PE_FLAG::PROTOCOL_ERROR);

    if (port.pe_flags.test(PE_FLAG::FORWARD_PRL_ERROR)) {
        return;
    }

    if (err == PRL_ERROR::RCH_SEND_FAIL ||
        err == PRL_ERROR::TCH_SEND_FAIL)
    {
        pe.change_state(PE_SNK_Send_Soft_Reset);
        return;
    }

    if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT) &&
        port.pe_flags.test(PE_FLAG::AMS_ACTIVE) &&
        !port.pe_flags.test(PE_FLAG::AMS_FIRST_MSG_SENT))
    {
        // Discard is not possible without an RX message, but let's check to be sure.
        if (port.pe_flags.test(PE_FLAG::MSG_RECEIVED)) {
            port.pe_flags.set(PE_FLAG::DO_SOFT_RESET_ON_UNSUPPORTED);
        }
        pe.change_state(PE_SNK_Ready);
        return;
    }

    pe.change_state(PE_SNK_Send_Soft_Reset);
}

void PE_EventListener::on_receive(const MsgToPe_PrlReportDiscard&) {
    PE_LOGI("=> Message discarded (from PRL)");
    pe.port.pe_flags.set(PE_FLAG::MSG_DISCARDED);
}

void PE_EventListener::on_receive(const MsgToPe_PrlSoftResetFromPartner&) {
    PE_LOGI("=> Soft Reset from port partner");
    if (pe.is_uninitialized()) { return; }
    if (pe.get_state_id() == PE_Src_Disabled) { return; }
    pe.change_state(PE_SNK_Soft_Reset);
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetFromPartner&) {
    PE_LOGI("=> Hard Reset from port partner");
    if (pe.is_uninitialized()) { return; }
    pe.change_state(PE_SNK_Transition_to_default);
}

void PE_EventListener::on_receive(const MsgToPe_PrlHardResetSent&) {
    if (pe.is_uninitialized()) { return; }
    pe.port.pe_flags.clear(PE_FLAG::PRL_HARD_RESET_PENDING);
}

void PE_EventListener::on_receive_unknown(ETL_MAYBE_UNUSED const etl::imessage& msg) {
    PE_LOGE("PE unknown message, id: {}", msg.get_message_id());
}

} // namespace pd
