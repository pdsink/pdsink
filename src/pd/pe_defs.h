#pragma once

namespace pd {

// Boolean flags to organize a simple DPM request queue
enum class DPM_REQUEST_FLAG {
    NONE, // Skip 0 to simplify active request checks
    NEW_POWER_LEVEL,
    EPR_MODE_ENTRY,

    _Count
};

enum class PE_FLAG {
    // Flags of message transfer (set by PRL)
    TX_COMPLETE,   // Message sent
    MSG_DISCARDED, // Outgoing message discarded by new incoming one
    MSG_RECEIVED,  // Got reply OR new message (which discarded outgoing transfer)

    // By default, a PRL error usually causes a soft reset (or a return to the
    // ready state). This flag can be set at state entry when custom handling is
    // needed. Control will then continue in the current state.
    FORWARD_PRL_ERROR,
    PROTOCOL_ERROR,

    HAS_EXPLICIT_CONTRACT,
    IN_EPR_MODE,
    AMS_ACTIVE,
    AMS_FIRST_MSG_SENT,
    EPR_AUTO_ENTER_DISABLED,
    // Used to mark the complete entry sequence at start, when the sink becomes
    // ready to accept DPM requests.
    // - [SELECT CAPABILITIES, [EPR ENTER, SELECT CAPABILITIES]]
    // The tail is optional, for EPR chargers only.
    HANDSHAKE_REPORTED,

    // Minor flags to control local behavior in states
    WAIT_DPM_TRANSIT_TO_DEFAULT,
    PRL_HARD_RESET_PENDING,
    HR_BY_CAPS_TIMEOUT,
    DO_SOFT_RESET_ON_UNSUPPORTED,
    CAN_SEND_SOFT_RESET,
    TRANSMIT_REQUEST_SUCCEEDED,

    _Count
};

} // namespace pd
