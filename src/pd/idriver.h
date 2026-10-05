#pragma once

#include <etl/delegate.h>

#include "pd_conf.h"

namespace pd {

enum class TCPC_POLARITY {
    CC1 = 0, // CC1 is active
    CC2 = 1, // CC2 is active
    NONE = 2, // Not selected yet
    TOGGLING = 3 // Hardware is looking for the active CC line
};

// Voltage ranges from comparator, corresponding to Rp values
namespace TCPC_CC_LEVEL {
    enum Type {
        NONE = 0,
        RP_0_5 = 1, // Rp (0.5A) active
        RP_1_5 = 2, // Rp (1.5A) active
        RP_3_0 = 3, // Rp (3.0A) active
    };

    constexpr uint8_t SinkTxNG = RP_1_5;
    constexpr uint8_t SinkTxOK = RP_3_0;
};

enum class TCPC_BIST_MODE {
    Off = 0,
    Carrier = 1,
    TestData = 2
};

// Hardware features used by the protocol and Type-C layers.
// Not final. Cases without hardware CRC support may need to be dropped.
struct TCPC_HW_FEATURES {
    bool rx_auto_goodcrc_send;
    bool tx_auto_goodcrc_check;
    bool toggling;
    bool tx_auto_retry;
};

// NOTE: discarding is done at PRL layer.
enum class TCPC_TRANSMIT_STATUS {
    // No operation
    UNSET = 0,
    // PRL prepared data for PHY
    ENQUEUED = 1,
    // PHY accepted data and started sending
    SENDING = 2,
    // Transmission is completed (and GoodCRC received, if supported)
    SUCCEEDED = 3,
    // Transmission failed (no GoodCRC received)
    FAILED = 4
};

static inline bool is_tcpc_transmit_in_progress(TCPC_TRANSMIT_STATUS status) {
    return status == TCPC_TRANSMIT_STATUS::ENQUEUED ||
           status == TCPC_TRANSMIT_STATUS::SENDING;
}

enum class TCPC_CC_REQ {
    CC1,
    CC2,
    CC1CC2,
    ACTIVE_CC
};

enum class TCPC_CC_GET {
    CC1,
    CC2,
    ACTIVE_CC
};

//
// Interfaces
//

class ITimer {
public:
    using TickSource = etl::delegate<uint32_t()>;

    // Source of cyclic 32-bit microsecond ticks. Deadline comparisons are valid
    // while the distance to or past a deadline is less than 2^31 microseconds.
    virtual TickSource get_tick_source() const = 0;

    // Arm a one-shot wakeup at an absolute tick value. A deadline that is
    // already due must still cause a wakeup.
    virtual void rearm(uint32_t deadline_ticks) = 0;
};

class ITCPC {
public:
    // Since TCPC hardware can be asynchronous (for example, connected via I2C
    // instead of direct memory mapping), commands go through several steps:
    //
    // 1. Send a command describing what to do (req_xxx).
    // 2. Monitor status, wait for completion (is_xxx_done or a getter's
    //    returned completion flag).
    // 3. Optionally, read fetched data (for example, CC line level).
    //

    // Request a cache update for the selected CC line(s).
    virtual void req_fetch_cc(TCPC_CC_REQ selector) = 0;
    virtual bool is_fetch_cc_done() const = 0;

    // Read the cached CC level, even while updating. Does not change polarity.
    virtual auto get_cc(TCPC_CC_GET selector) const -> TCPC_CC_LEVEL::Type = 0;

    // Spec requires VBUS detection. While we can use CC1/CC2 instead,
    // keep this method for compatibility.
    virtual bool is_vbus_ok() = 0;

    // Apply polarity or start hardware CC detection.
    virtual void req_set_polarity(TCPC_POLARITY active_cc) = 0;
    virtual bool is_set_polarity_done() = 0;
    // Actual polarity; TOGGLING ends only when CC data is ready.
    virtual auto get_polarity() const -> TCPC_POLARITY = 0;

    // Always flush TX. Flush pending RX on disable and before enabling from
    // disabled; preserve it on repeated enable.
    virtual void req_rx_enable(bool enable) = 0;
    virtual bool is_rx_enable_done() = 0;

    // Fetch pending RX data.
    virtual bool fetch_rx_data() = 0;

    // Transmit the packet in tx_info
    virtual void req_transmit() = 0;

    // Set BIST mode
    virtual void req_set_bist(TCPC_BIST_MODE mode) = 0;
    virtual bool is_set_bist_done() = 0;
    // Last applied mode; pending requests do not change it.
    virtual auto get_bist_mode() const -> TCPC_BIST_MODE = 0;

    virtual void req_hr_send() = 0;
    virtual bool is_hr_send_done() = 0;

    virtual auto get_hw_features() -> TCPC_HW_FEATURES = 0;
};

class IDriver: public ITCPC, public ITimer {
public:
    virtual void setup() = 0;

    // Wake up the PD event loop, ensuring it runs in the appropriate execution
    // context. Use this to signal the PD event loop from application code through
    // DPM or PE flags.
    virtual void wakeup() = 0;
};

} // namespace pd
