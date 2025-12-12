#pragma once

enum class PRL_ERROR {
    // The spec describes PE reactions to errors poorly. Let's keep all
    // details in the report to have more flexibility in the error handler.

    RCH_BAD_SEQUENCE,       // wrong input chunk (for chunked messages)
    RCH_SEND_FAIL,          // failed to request next chunk (no GoodCRC)
    RCH_SEQUENCE_DISCARDED, // new message interrupted sequence
    RCH_SEQUENCE_TIMEOUT,   // no response for chunk request

    TCH_ENQUEUE_DISCARDED,  // RCH busy, TCH can't accept new message
    TCH_BAD_SEQUENCE,
    TCH_SEND_FAIL,
    TCH_DISCARDED,
    TCH_SEQUENCE_TIMEOUT,
};

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
