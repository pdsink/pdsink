#pragma once

#include <etl/message.h>

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
DEFINE_PARAM_MSG(MsgTask_Timer, MSG_TASK_TIMER, uint64_t, timestamp);

//
// DPM notification messages
//
enum msg_dpm_id {
    // Generic events
    MSG_DPM_STARTUP = 200,
    MSG_DPM_TRANSIT_TO_DEFAULT,
    MSG_DPM_SRC_CAPS_RECEIVED,
    MSG_DPM_SELECT_CAP_DONE,
    MSG_DPM_SRC_DISABLED,
    MSG_DPM_ALERT,
    MSG_DPM_EPR_ENTRY_FAILED,
    // Results of DPM requests
    MSG_DPM_NEW_POWER_LEVEL,
    MSG_DPM_PPS_STATUS,
    MSG_DPM_PARTNER_REVISION,
    MSG_DPM_SOURCE_INFO,
    // Sugar
    MSG_DPM_HANDSHAKE_DONE, // Contracted and upgraded to EPR if possible.
    MSG_DPM_IDLE, // PE_SNK_Ready with no activity.
};

DEFINE_SIMPLE_MSG(MsgDpm_Startup, MSG_DPM_STARTUP);
DEFINE_SIMPLE_MSG(MsgDpm_TransitToDefault, MSG_DPM_TRANSIT_TO_DEFAULT);
DEFINE_SIMPLE_MSG(MsgDpm_SrcCapsReceived, MSG_DPM_SRC_CAPS_RECEIVED);
DEFINE_SIMPLE_MSG(MsgDpm_SelectCapDone, MSG_DPM_SELECT_CAP_DONE);
DEFINE_SIMPLE_MSG(MsgDpm_SrcDisabled, MSG_DPM_SRC_DISABLED);
DEFINE_PARAM_MSG(MsgDpm_Alert, MSG_DPM_ALERT, uint32_t, value);
DEFINE_PARAM_MSG(MsgDpm_EPREntryFailed, MSG_DPM_EPR_ENTRY_FAILED, uint32_t, reason);
DEFINE_PARAM_MSG(MsgDpm_NewPowerLevel, MSG_DPM_NEW_POWER_LEVEL, bool, ok);
DEFINE_PARAM_MSG(MsgDpm_PPSStatus, MSG_DPM_PPS_STATUS, uint32_t, value);
DEFINE_PARAM_MSG(MsgDpm_PartnerRevision, MSG_DPM_PARTNER_REVISION, uint32_t, value);
DEFINE_PARAM_MSG(MsgDpm_SourceInfo, MSG_DPM_SOURCE_INFO, uint32_t, value);
DEFINE_SIMPLE_MSG(MsgDpm_HandshakeDone, MSG_DPM_HANDSHAKE_DONE);
DEFINE_SIMPLE_MSG(MsgDpm_Idle, MSG_DPM_IDLE);

} // namespace pd
