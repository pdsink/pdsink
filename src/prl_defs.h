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

namespace PRL_FLAG {
    enum Type {
        // This is implemented according to spec for consistency, but not
        // actually used in public API
        ABORT,
        FLAGS_COUNT
    };
}; // namespace PRL_FLAG

namespace RCH_FLAG {
    enum Type {
        RX_ENQUEUED, // From RX
        RCH_ERROR_PENDING,
        FLAGS_COUNT
    };
}; // namespace RCH_FLAG

namespace TCH_FLAG {
    enum Type {
        MSG_ENQUEUED, // From PE
        NEXT_CHUNK_REQUEST,
        TCH_ERROR_PENDING,
        FLAGS_COUNT
    };
}; // namespace TCH_FLAG

namespace PRL_TX_FLAG {
    enum Type {
        TX_CHUNK_ENQUEUED, // From TCH/RCH
        TX_COMPLETED,
        TX_DISCARDED,
        TX_ERROR,
        START_OF_AMS_DETECTED,
        FLAGS_COUNT
    };
}; // namespace PRL_TX_FLAG

namespace PRL_HR_FLAG {
    enum Type {
        HARD_RESET_FROM_PARTNER,
        HARD_RESET_FROM_PE,
        PE_HARD_RESET_COMPLETE,
        FLAGS_COUNT
    };
}; // namespace PRL_HR_FLAG
