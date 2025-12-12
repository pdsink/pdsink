#pragma once

#include "pd_conf.h"

namespace pd {

enum class TCPC_POLARITY {
    CC1 = 0, // CC1 is active
    CC2 = 1, // CC2 is active
    NONE = 2 // Not selected yet
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

// Hardware features description to clarify Rx/Tx logic in PRL.
// Not final. Cases without hardware CRC support may need to be dropped.
struct TCPC_HW_FEATURES {
    bool rx_auto_goodcrc_send;
    bool tx_auto_goodcrc_check;
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

//
// Interfaces
//

class ITimer {
public:
    using TimeFunc = uint32_t(*)();
    virtual TimeFunc get_time_func() const = 0;
    // Set the interval (from "now") of the next timer tick. For a simple
    // implementation, make this a dummy and tick every 1 ms.
    virtual void rearm(uint32_t interval) = 0;

    virtual bool is_rearm_supported() = 0;
};

// TODO: Seems all modern chips support auto-toggle.
// Consider adding it to the API (with optional emulation) and remove the manual
// method from TC.
class ITCPC {
public:
    // Since TCPC hardware can be asynchronous (for example, connected via I2C
    // instead of direct memory mapping), commands go through several steps:
    //
    // 1. Send a command describing what to do (req_xxx).
    // 2. Monitor status, wait for completion (is_xxx_done).
    // 3. Optionally, read fetched data (for example, CC line level).
    //

    // Request to fetch both CC1/CC2 line levels. This may be slow because
    // switches block measurement. Used for manual polarity detection only.
    virtual void req_scan_cc() = 0;
    virtual bool try_scan_cc_result(TCPC_CC_LEVEL::Type& cc1, TCPC_CC_LEVEL::Type& cc2) = 0;

    // Used only for SinkTxOK waiting in the 3.0 protocol. Possible glitches
    // caused by BMC are not critical here. Debounced polling is OK because
    // transfer locks are very rare and short.
    virtual void req_active_cc() = 0;
    virtual bool try_active_cc_result(TCPC_CC_LEVEL::Type& cc) = 0;

    // Spec requires VBUS detection. While we can use CC1/CC2 instead,
    // keep this method for compatibility.
    virtual bool is_vbus_ok() = 0;

    // NOTE: Any other actions should NOT reset the selected polarity. It is
    // updated only by this call when a new cable connection is detected.
    virtual void req_set_polarity(TCPC_POLARITY active_cc) = 0;
    virtual bool is_set_polarity_done() = 0;

    // NOTE: Disable should flush the RX/TX FIFOs, and enable should flush the
    // TX FIFO only.
    virtual void req_rx_enable(bool enable) = 0;
    virtual bool is_rx_enable_done() = 0;

    // Fetch pending RX data.
    virtual bool fetch_rx_data() = 0;

    // Transmit the packet in tx_info
    virtual void req_transmit() = 0;

    // Set BIST mode
    virtual void req_set_bist(TCPC_BIST_MODE mode) = 0;
    virtual bool is_set_bist_done() = 0;

    virtual void req_hr_send() = 0;
    virtual bool is_hr_send_done() = 0;

    virtual auto get_hw_features() -> TCPC_HW_FEATURES = 0;
};

class IDriver: public ITCPC, public ITimer {
public:
    virtual void setup() = 0;
};

} // namespace pd
