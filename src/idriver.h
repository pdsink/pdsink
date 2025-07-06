#pragma once

#include "data_objects.h"
#include "utils/atomic_bits.h"

#include <etl/delegate.h>
#include <etl/message_router.h>
#include <etl/utility.h>
#include <etl/vector.h>

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

// Those flags display pending TCPC calls. Some flags can be not necessary,
// when TCPC is memory mapped instead of I2C.
namespace TCPC_CALL_FLAG {
    enum Type {
        REQ_CC_BOTH,
        REQ_CC_ACTIVE,
        SET_POLARITY,
        SET_RX_ENABLE,
        TRANSMIT,
        BIST_CARRIER_ENABLE,
        HARD_RESET,
        FLAGS_COUNT
    };
};

using TCPC_STATE = AtomicBits<TCPC_CALL_FLAG::FLAGS_COUNT>;

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
// TCPC event messages
//
class MsgTcpcHardReset : public etl::message<0> {};

class MsgTcpcWakeup : public etl::message<1> {};

class MsgTcpcTransmitStatus : public etl::message<2> {
public:
    explicit MsgTcpcTransmitStatus(TCPC_TRANSMIT_STATUS::Type status) : status{status} {}
    TCPC_TRANSMIT_STATUS::Type status;
};

class MsgTimerEvent : public etl::message<3> {};

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

class ITCPC {
public:
    // Return bitfield of flags to track stacked commands
    virtual TCPC_STATE& get_state() = 0;

    // Since TCPC hardware can be async (connected via i2c instead of direct
    // memory mapping), all commands go via several steps:
    //
    // 1. Send command (request) and wait until complete
    // 2. Monitor state flags, to wait for complete
    // 3. Read fetched data, if expected (for example, CC line level)
    //

    // Request to fetch both CC1/CC2 lines levels. May be slow, because switches
    // measurement block. Used for initial connection/polarity detection only.
    virtual void req_cc_both() = 0;

    // After polarity detected, measurement block is attached to active
    // CC line instantly. Such requests are fast, because no
    // measurement delay required. Active CC levels used for 2 cases:
    //
    // 1. When 3.0 Sink waits for AMS allowed (1.5A source to switch to 3.0A)
    // 2. Monitor detach (active CC become zero)
    //
    // Note, detach is polled by TCPC internally. Direct call of this function
    // required for 3.0+ Sink transfers only, to sync last value.
    virtual void req_cc_active() = 0;

    // Get fetched data after req_cc_both/req_cc_secondary completed.
    virtual TCPC_CC_LEVEL::Type get_cc(TCPC_CC::Type cc) = 0;

    // Note, any other actions should NOT reset selected polarity. It's updated
    // only by this call, when new cable connect detected.
    virtual void set_polarity(TCPC_POLARITY::Type active_cc) = 0;

    // Note, this should flush RX/TX FIFO on disable,
    // and TX FIFO (only) on enable.
    virtual void set_rx_enable(bool enable) = 0;

    // True when new packet is available in rx_info.
    virtual bool has_rx_data() = 0;
    // Fetch pending RX data.
    virtual bool fetch_rx_data(PD_CHUNK& data) = 0;

    // Transmit packet in tx_info
    virtual void transmit(const PD_CHUNK& tx_info) = 0;

    // On/off BIST carrier
    virtual void bist_carrier_enable(bool enable) = 0;

    virtual void hard_reset() = 0;

    virtual auto get_hw_features() -> TCPC_HW_FEATURES = 0;
};

class IDriver: public ITCPC, public ITimer {
public:
    virtual void start() = 0;
    virtual void set_msg_router(etl::imessage_router& router) = 0;
};

} // namespace pd
