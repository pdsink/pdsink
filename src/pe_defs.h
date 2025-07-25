#pragma once

namespace pd {

// Boolean flags to organize  simple DPM requests queue
namespace DPM_REQUEST_FLAG {
    enum Type {
        UNUSED, // Skip 0 to simplify current request checks
        NEW_POWER_LEVEL,
        EPR_MODE_ENTRY,
        GET_PPS_STATUS,
        GET_SRC_INFO,
        GET_REVISION,

        REQUEST_COUNT
    };
} // namespace DPM_REQUEST_FLAG

namespace PE_FLAG {
    enum Type {
        // Flags of message transfer (set by PRL)
        TX_COMPLETE,   // Message sent
        MSG_DISCARDED, // Outgoing message discarded by new incoming one
        MSG_RECEIVED,  // Got reply OR new message (which discarded outgoing transfer)

        // By default PRL error usually causes soft reset. This flag can be set
        // at state entry when custom handling needed. Then error will rise
        // PROTOCOL_ERROR flag instead.
        FORWARD_PRL_ERROR,
        PROTOCOL_ERROR,

        HAS_EXPLICIT_CONTRACT,
        IN_EPR_MODE,
        AMS_ACTIVE,
        AMS_FIRST_MSG_SENT,
        EPR_AUTO_ENTER_DISABLED,

        // Minor local flags to control states behaviour
        WAIT_DPM_TRANSIT_TO_DEFAULT,
        PRL_HARD_RESET_PENDING,
        IS_FROM_EVALUATE_CAPABILITY,
        HR_BY_CAPS_TIMEOUT,
        DO_SOFT_RESET_ON_UNSUPPORTED,
        CAN_SEND_SOFT_RESET,
        TRANSMIT_REQUEST_SUCCEEDED,

        FLAGS_COUNT
    };
} // namespace PE_FLAG

} // namespace pd
