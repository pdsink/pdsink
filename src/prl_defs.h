#pragma once

enum class PRL_ERROR {
    // Spec has too poor description of PE reactions on errors. Let's keep all
    // details in report, to have more flexibility in error handler.

    RCH_BAD_SEQUENCE,       // wrong input chunk (for chunked messages)
    RCH_SEND_FAIL,          // failed to request next chunk (no GoodCRC)
    RCH_SEQUENCE_DISCARDED, // new message interrupted sequence
    RCH_SEQUENCE_TIMEOUT,   // no response for chunk request

    TCH_ENQUIRE_DISCARDED,  // RCH busy, TCH can't accept new message
    TCH_BAD_SEQUENCE,
    TCH_SEND_FAIL,
    TCH_DISCARDED,
    TCH_SEQUENCE_TIMEOUT,
};

enum class PRL_FLAG {
    // This is implemented according to spec for consistency, but not
    // actually used in public API
    ABORT,
    _Count
};

enum class RCH_FLAG {
    RX_ENQUEUED, // From RX
    RCH_ERROR_PENDING,
    _Count
};

enum class TCH_FLAG {
    MSG_ENQUEUED, // From PE
    NEXT_CHUNK_REQUEST,
    TCH_ERROR_PENDING,
    _Count
};

enum class PRL_TX_FLAG {
    TX_CHUNK_ENQUEUED, // From TCH/RCH
    TX_COMPLETED,
    TX_DISCARDED,
    TX_ERROR,
    START_OF_AMS_DETECTED,
    _Count
};

enum class PRL_HR_FLAG {
    HARD_RESET_FROM_PARTNER,
    HARD_RESET_FROM_PE,
    PE_HARD_RESET_COMPLETE,
    _Count
};
