#pragma once

#include <etl/message.h>

#include "idriver.h"
#include "prl_defs.h"

namespace pd {

#define DEFINE_SIMPLE_MSG(ClassName, MsgId) \
    class ClassName : public etl::message<MsgId> {}

#define DEFINE_PARAM_MSG(ClassName, MsgId, ParamType, ParamName) \
    class ClassName : public etl::message<MsgId> { \
    public: \
        explicit ClassName(ParamType ParamName) : ParamName{ParamName} {} \
        const ParamType ParamName; \
    }

//
// Generic messages
//

enum msg_sys_id {
    MSG_FSM_TRANSIT = 10,
    MSG_SYS_UPDATE,
    MSG_TASK_WAKEUP,
    MSG_TASK_TIMER
};

// Used to force FSM-s state change
DEFINE_PARAM_MSG(MsgTransitTo, MSG_FSM_TRANSIT, int, state_id);
// Dummy event for cascade components call
DEFINE_SIMPLE_MSG(MsgSysUpdate, MSG_SYS_UPDATE);

// Task (event loop) kicks
DEFINE_SIMPLE_MSG(MsgTask_Wakeup, MSG_TASK_WAKEUP);
DEFINE_SIMPLE_MSG(MsgTask_Timer, MSG_TASK_TIMER);

//
// DPM notification messages
//
enum msg_dpm_id {
    // Generic events
    MSG_TO_DPM__STARTUP = 50,
    MSG_TO_DPM__TRANSIT_TO_DEFAULT,
    MSG_TO_DPM__SRC_CAPS_RECEIVED,
    MSG_TO_DPM__SELECT_CAP_DONE,
    MSG_TO_DPM__SRC_DISABLED,
    MSG_TO_DPM__ALERT,
    MSG_TO_DPM__EPR_ENTRY_FAILED,
    // Results of DPM requests
    MSG_TO_DPM__NEW_POWER_LEVEL,
    MSG_TO_DPM__PPS_STATUS,
    MSG_TO_DPM__PARTNER_REVISION,
    MSG_TO_DPM__SOURCE_INFO,
    // Sugar
    MSG_TO_DPM__HANDSHAKE_DONE, // Contracted and upgraded to EPR if possible.
    MSG_TO_DPM__IDLE, // PE_SNK_Ready with no activity.
};

DEFINE_SIMPLE_MSG(MsgToDpm_Startup, MSG_TO_DPM__STARTUP);
DEFINE_SIMPLE_MSG(MsgToDpm_TransitToDefault, MSG_TO_DPM__TRANSIT_TO_DEFAULT);
DEFINE_SIMPLE_MSG(MsgToDpm_SrcCapsReceived, MSG_TO_DPM__SRC_CAPS_RECEIVED);
DEFINE_SIMPLE_MSG(MsgToDpm_SelectCapDone, MSG_TO_DPM__SELECT_CAP_DONE);
DEFINE_SIMPLE_MSG(MsgToDpm_SrcDisabled, MSG_TO_DPM__SRC_DISABLED);
DEFINE_PARAM_MSG(MsgToDpm_Alert, MSG_TO_DPM__ALERT, uint32_t, value);
DEFINE_PARAM_MSG(MsgToDpm_EPREntryFailed, MSG_TO_DPM__EPR_ENTRY_FAILED, uint32_t, reason);
DEFINE_PARAM_MSG(MsgToDpm_NewPowerLevel, MSG_TO_DPM__NEW_POWER_LEVEL, bool, ok);
DEFINE_PARAM_MSG(MsgToDpm_PPSStatus, MSG_TO_DPM__PPS_STATUS, uint32_t, value);
DEFINE_PARAM_MSG(MsgToDpm_PartnerRevision, MSG_TO_DPM__PARTNER_REVISION, uint32_t, value);
DEFINE_PARAM_MSG(MsgToDpm_SourceInfo, MSG_TO_DPM__SOURCE_INFO, uint32_t, value);
DEFINE_SIMPLE_MSG(MsgToDpm_HandshakeDone, MSG_TO_DPM__HANDSHAKE_DONE);
DEFINE_SIMPLE_MSG(MsgToDpm_Idle, MSG_TO_DPM__IDLE);

//
// PE incoming messates
//
enum msg_pe_id {
    MSG_TO_PE__PRL_MESSAGE_RECEIVED = 100,
    MSG_TO_PE__PRL_MESSAGE_SENT,
    MSG_TO_PE__PRL_REPORT_ERROR,
    MSG_TO_PE__PRL_REPORT_DISCARD,
    MSG_TO_PE__PRL_SOFT_RESET_FROM_PARTNER,
    MSG_TO_PE__PRL_HARD_RESET_FROM_PARTNER,
    MSG_TO_PE__PRL_HARD_RESET_SENT,
};

DEFINE_SIMPLE_MSG(MsgToPe_PrlMessageReceived, MSG_TO_PE__PRL_MESSAGE_RECEIVED);
DEFINE_SIMPLE_MSG(MsgToPe_PrlMessageSent, MSG_TO_PE__PRL_MESSAGE_SENT);
DEFINE_PARAM_MSG(MsgToPe_PrlReportError, MSG_TO_PE__PRL_REPORT_ERROR, PRL_ERROR, error);
DEFINE_SIMPLE_MSG(MsgToPe_PrlReportDiscard, MSG_TO_PE__PRL_REPORT_DISCARD);
DEFINE_SIMPLE_MSG(MsgToPe_PrlSoftResetFromPartner, MSG_TO_PE__PRL_SOFT_RESET_FROM_PARTNER);
DEFINE_SIMPLE_MSG(MsgToPe_PrlHardResetFromPartner, MSG_TO_PE__PRL_HARD_RESET_FROM_PARTNER);
DEFINE_SIMPLE_MSG(MsgToPe_PrlHardResetSent, MSG_TO_PE__PRL_HARD_RESET_SENT);

//
// PRL incoming messages
//
enum msg_prl_id {
    MSG_TO_PRL__SOFT_RESET_FROM_PE = 120,
    MSG_TO_PRL__HARD_RESET_FROM_PE,
    MSG_TO_PRL__PE_HARD_RESET_DONE,
    MSG_TO_PRL__TCPC_HARD_RESET,
    MSG_TO_PRL__TCPC_TRANSMIT_STATUS,
};

DEFINE_SIMPLE_MSG(MsgToPrl_SoftResetFromPe, MSG_TO_PRL__SOFT_RESET_FROM_PE);
DEFINE_SIMPLE_MSG(MsgToPrl_HardResetFromPe, MSG_TO_PRL__HARD_RESET_FROM_PE);
DEFINE_SIMPLE_MSG(MsgToPrl_PEHardResetDone, MSG_TO_PRL__PE_HARD_RESET_DONE);
DEFINE_SIMPLE_MSG(MsgToPrl_TcpcHardReset, MSG_TO_PRL__TCPC_HARD_RESET);
DEFINE_PARAM_MSG(MsgToPrl_TcpcTransmitStatus, MSG_TO_PRL__TCPC_TRANSMIT_STATUS, TCPC_TRANSMIT_STATUS::Type, status);

} // namespace pd
