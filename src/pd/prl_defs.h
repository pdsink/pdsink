#pragma once

enum class RCH_FLAG {
    RX_ENQUEUED, // From RX
    _Count
};

enum class TCH_FLAG {
    MSG_FROM_PE_ENQUEUED,
    CHUNK_FROM_RX,
    _Count
};

enum class PRL_TX_FLAG {
    START_OF_AMS_DETECTED,
    // Input signal from RCH/TCH
    TX_CHUNK_ENQUEUED,
    // Output signal for RCH/TCH
    TX_COMPLETED,
    TX_ERROR,
    _Count
};

enum class PRL_HR_FLAG {
    HARD_RESET_FROM_PARTNER,
    HARD_RESET_FROM_PE,
    PE_HARD_RESET_COMPLETE,
    _Count
};
