#pragma once

namespace pd {

namespace TCPC_CC {
    enum Type {
        CC1 = 0,
        CC2 = 1,
        ACTIVE // For selected polarity
    };
};

namespace TCPC_POLARITY {
    enum Type {
        CC1 = 0, // CC1 is active
        CC2 = 1, // CC2 is active
        NONE = 2 // Not selected yet
    };
};

// Voltage ranges from comparator, correcpoinding to Rp values
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

// Hardware features description, to clarify Rx/Tx logic in PRL.
// Not final. May be cases without hardware CRC support should be dropped.
struct TCPC_HW_FEATURES {
    bool rx_goodcrc_send;
    bool tx_goodcrc_receive;
    bool tx_retransmit;
    bool cc_update_event;
    bool unchunked_ext_msg; // By default - not supported
};

// All PD chips have automatic GoodCRC handling. These states are
// enhanced over spec requirements, to cover all possible hardware use cases.
// Details will depend on concrete driver implementation.
namespace TCPC_TRANSMIT_STATUS {
    enum Type {
        // No operation
        UNSET = -1,
        // Transmission is in progress (including I2C transfer)
        WAITING = 0,
        // Transmission is completed (and GoodCRC received, if supported)
        SUCCEEDED = 1,
        // Transmission failed (no GoodCRC received)
        FAILED = 2,
        // Transmit discarded by by new incoming packet.
        DISCARDED = 3,
    };
};

namespace SOP_TYPE {
    enum Type {
        SOP = 0,
        SOP_PRIME = 1,
        SOP_PRIME_PRIME = 2,
        SOP_DEBUG_PRIME = 3,
        SOP_DEBUG_PRIME_PRIME = 4,
        INVALID = 0xF
    };
}

//
// Interfaces
//

class ITimer {
public:
    // You can actually return 32-bit value, if overflow after 49.7 days is
    // acceptable.
    virtual uint64_t get_timestamp() = 0;
    // Set interval (from "now") of next timer tick. For simple implementation
    // make this dummy and tick every 1 ms.
    virtual void rearm(uint32_t interval) = 0;

    virtual bool is_rearm_supported() = 0;
};

// TODO: Seems all modern chips support auto-toggle.
// Consider add it to api (with optional emulation) and remove manual method
// from TC.
class ITCPC {
public:
    // Since TCPC hardware can be async (connected via i2c instead of direct
    // memory mapping), all commands go via several steps:
    //
    // 1. Send command what to do (req_xxx)
    // 2. Monitor status, wait for complete (is_xxx_done)
    // 3. Optionally, read fetched data (for example, CC line level)
    //

    // Request to fetch both CC1/CC2 lines levels. May be slow, because switches
    // measurement block. Used for manual polarity detection only.
    virtual void req_scan_cc() = 0;
    virtual bool is_scan_cc_done() = 0;

    // Used only for SinkTxOK waiting in 3.0 protocol. Possible glitches,
    // caused by BMC are not critical here. Debounced polling is ok, because
    // transfer locks are very rare and short.
    virtual void req_active_cc() = 0;
    virtual bool is_active_cc_done() = 0;

    // Get fetched data after req_scan_cc/req_active_cc completed.
    virtual TCPC_CC_LEVEL::Type get_cc(TCPC_CC::Type cc) = 0;

    // Spec requires VBUS detection. While we can use CC1/CC2 instead,
    // keep this method for compatibility.
    virtual bool is_vbus_ok() = 0;

    // Note, any other actions should NOT reset selected polarity. It's updated
    // only by this call, when new cable connect detected.
    virtual void req_set_polarity(TCPC_POLARITY::Type active_cc) = 0;
    virtual bool is_set_polarity_done() = 0;

    // Note, this should flush RX/TX FIFO on disable,
    // and TX FIFO (only) on enable.
    virtual void req_rx_enable(bool enable) = 0;
    virtual bool is_rx_enable_done() = 0;

    // Fetch pending RX data.
    virtual bool fetch_rx_data() = 0;

    // Transmit packet in tx_info
    virtual void req_transmit() = 0;
    virtual bool is_transmit_done() = 0;

    // On/off BIST carrier
    virtual void req_bist_carrier_enable(bool enable) = 0;
    virtual bool is_bist_carrier_enable_done() = 0;

    virtual void req_hr_send() = 0;
    virtual bool is_hr_send_done() = 0;

    virtual auto get_hw_features() -> TCPC_HW_FEATURES = 0;
};

class IDriver: public ITCPC, public ITimer {
public:
    virtual void setup() = 0;
};

} // namespace pd
