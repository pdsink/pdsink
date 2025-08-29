#pragma once

namespace pd {

// Boolean flags to organize  simple DPM requests queue
enum class DPM_REQUEST_FLAG {
    UNUSED, // Skip 0 to simplify current request checks
    NEW_POWER_LEVEL,
    EPR_MODE_ENTRY,

    _Count
};

enum class PE_FLAG {
    // Flags of message transfer (set by PRL)
    TX_COMPLETE,   // Message sent
    MSG_DISCARDED, // Outgoing message discarded by new incoming one
    MSG_RECEIVED,  // Got reply OR new message (which discarded outgoing transfer)

    // By default PRL error usually causes soft reset (or return to ready state).
    // This flag can be set at state entry when custom handling needed.
    // Then control will continue in the current state.
    FORWARD_PRL_ERROR,
    PROTOCOL_ERROR,

    HAS_EXPLICIT_CONTRACT,
    IN_EPR_MODE,
    AMS_ACTIVE,
    AMS_FIRST_MSG_SENT,
    EPR_AUTO_ENTER_DISABLED,

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
