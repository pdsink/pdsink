#pragma once

#include "common.h"

#include <etl/message.h>
#include <etl/vector.h>

#include <stdint.h>

namespace pd {

constexpr int MaxExtendedMsgLen = 260;
constexpr int MaxExtendedMsgChunkLen = 26;
constexpr int MaxExtendedMsgLegacyLen = 26;
constexpr int MaxChunksPerMsg = 10;

constexpr int MaxPdoObjects = 11; // 7 for SPR, 11 for EPR.
constexpr int MaxUnchunkedMsgLen = 28;

using PDO_LIST = etl::vector<uint32_t, MaxPdoObjects>;

// Internal top level event bitmasks, for main event loop.
namespace PD_EVENT {
    enum : uint32_t {
        // Generated by timer timeouts or periodic timer
        // (depends on driver implementation)
        TIMER = BIT(0),
        // Dummy flag for hand-generated notifications
        WAKEUP = BIT(1),
        // Event by hardware driver, to detect connector insert/eject
        DRV_UPDATE = BIT(2),
    };
}; // namespace PD_EVENT

class MsgPdEvents : public etl::message<0> {
public:
    explicit MsgPdEvents(uint32_t events) : value{events} {}
    uint32_t value;
    inline bool has_timeout() const { return value & PD_EVENT::TIMER; }
};

class MsgTransitTo : public etl::message<1> {
public:
    explicit MsgTransitTo(int state) : state_id{state} {}
    const int state_id;
};


namespace PD_PACKET_TYPE {
    enum Type {
        SOP = 0,
        SOP_PRIME = 1,
        SOP_PRIME_PRIME = 2,
        SOP_DEBUG_PRIME = 3,
        SOP_DEBUG_PRIME_PRIME = 4,

        INVALID = 0xf
    };
}; // namespace PD_PACKET_TYPE


// 6.3 Control Message
// Table 6.5 Control Message Types
namespace PD_CTRL_MSGT {
    enum Type {
        GoodCRC = 1,
        GotoMin = 2, // Deprecated
        Accept = 3,
        Reject = 4,
        Ping = 5, // Deprecated
        PS_RDY = 6,
        Get_Source_Cap = 7,
        Get_Sink_Cap = 8,
        DR_Swap = 9,
        PR_Swap = 10,
        VCONN_Swap = 11,
        Wait = 12,
        Soft_Reset = 13,
        // rev 3+
        Data_Reset = 14,
        Data_Reset_Complete = 15,
        Not_Supported = 16,
        Get_Source_Cap_Ext = 17,
        Get_Status = 18,
        FR_Swap = 19,
        Get_PPS_Status = 20,
        Get_Country_Codes = 21,
        Get_Sink_Cap_Extended = 22,
        Get_Source_Info = 23,
        Get_Revision = 24,
        // Reserved 25-31
    };
}; // namespace PD_CTRL_MSGT


// 6.4 Data Message
// Table 6.6 Data Message Types
namespace PD_DATA_MSGT {
    enum Type {
        Source_Capabilities = 1,
        Request = 2,
        BIST = 3,
        Sink_Capabilities = 4,
        Battery_Status = 5,
        Alert = 6,
        Get_Country_Info = 7,
        Enter_USB = 8,
        EPR_Request = 9,
        EPR_Mode = 10,
        Source_Info = 11,
        Revision = 12,
        // Reserved 13-14
        Vendor_Defined = 15,
        // Reserved 16-31
    };
}; // namespace PD_DATA_MSGT


// 6.5 Extended Message
// Table 6.53 Extended Message Types
namespace PD_EXT_MSGT {
    enum Type {
        // 0 reserved
        Source_Capabilities_Extended = 1,
        Status = 2,
        Get_Battery_Cap = 3,
        Get_Battery_Status = 4,
        Battery_Capabilities = 5,
        Get_Manufacturer_Info = 6,
        Manufacturer_Info = 7,
        Security_Request = 8,
        Security_Response = 9,
        Firmware_Update_Request = 10,
        Firmware_Update_Response = 11,
        PPS_Status = 12,
        Country_Info = 13,
        Country_Codes = 14,
        Sink_Capabilities_Extended = 15,
        Extended_Control = 16,
        EPR_Source_Capabilities = 17,
        EPR_Sink_Capabilities = 18,
        // Reserved 19-29
        Vendor_Defined_Extended = 30,
        // Reserved 31
    };
};


// 6.2.1.1 Message Header
union PD_HEADER {
    struct {
        uint16_t message_type : 5;
        uint16_t port_data_role : 1; // SOP only, Reserved for SOP'/SOP"
        uint16_t spec_revision : 2;
        uint16_t port_power_role : 1; // SOP only, Cable plug for SOP'/SOP"
        uint16_t message_id : 3;
        uint16_t data_obj_count : 3; // Number of data objects in the message
        uint16_t extended : 1;
    };
    uint16_t raw_value;
};


// 6.2.1.2 Extended Message Header
union PD_EXT_HEADER {
    struct {
        uint16_t data_size : 9;
        uint16_t : 1; // reserved
        uint16_t request_chunk : 1;
        uint16_t chunk_number : 4;
        uint16_t chunked : 1;
    };
    uint16_t raw_value;
};


namespace PD_REVISION {
    enum Type {
        REV10 = 0,
        REV20 = 1,
        REV30 = 2
    };
}; // namespace PD_REVISION


//
// [rev3.2] 6.4.1.1 Power Data Objects
//
// Only actually useable ones defined below
//

// Bits 30..31
namespace PDO_TYPE {
    enum {
        FIXED = 0,
        AUGMENTED = 3
    };
}; // namespace PDO_TYPE

// Bits 28..29 for APDO
namespace PDO_AUGMENTED_SUBTYPE {
    enum {
        SPR_PPS = 0,
        // Ones below are not supported in chargers yet (at the start of 2024)
        EPR_AVS = 1,
        SPR_AVS = 2
    };
}; // namespace PDO_AUGMENTED_SUBTYPE

// [rev3.2] 6.4.1.2.1 Fixed Supply Power Data Object
// Table 6.9
union PDO_FIXED {
    struct {
        uint32_t max_current : 10; // 10ma step
        uint32_t voltage : 10; // 50mv step
        uint32_t peak_current: 2; // ??
        uint32_t : 1;
        uint32_t epr_capable : 1;
        uint32_t unchunked_ext_msg_supported : 1;
        uint32_t dual_role_data : 1;
        uint32_t usb_comms_capable : 1;
        uint32_t unconstrained_power : 1;
        uint32_t usb_suspend_supported : 1; // For SNK RDO means "Higher Capability"
        uint32_t dual_role_power : 1;
        uint32_t pdo_type : 2;
    };
    uint32_t raw_value;
};

// [rev3.2] 6.4.1.2.4 Augmented Power Data Object (APDO)
// Table 6.13
union PDO_SPR_PPS {
    struct {
        uint32_t max_current: 7; // 50ma step
        uint32_t : 1;
        uint32_t min_voltage : 8; // 100mv step
        uint32_t : 1;
        uint32_t max_voltage : 8; // 100mv step
        uint32_t : 2;
        uint32_t pps_power_limited : 1;
        uint32_t apdo_subtype : 2; // 00b
        uint32_t pdo_type : 2; // 11b
    };
    uint32_t raw_value;
};

// [rev3.2] Table 6.15
union PDO_EPR_AVS {
    struct {
        uint32_t pdp: 8; // 1W step
        uint32_t min_voltage : 8; // 100mv step
        uint32_t : 1;
        uint32_t max_voltage : 9; // 100mv step
        uint32_t peak_current : 2;
        uint32_t apdo_subtype : 2; // 01b
        uint32_t pdo_type : 2; // 11b
    };
    uint32_t raw_value;
};

//
// Request Data Objects ([rev3.2] 6.4.2 Request Message)
//
// Only actually useable ones defined below
//

// [rev3.2] Table 6.23 Fixed and Variable Request Data Object
union RDO_FIXED {
    struct {
        uint32_t max_current : 10; // 10ma step
        uint32_t operating_current : 10; // 10ma step
        uint32_t : 2;
        uint32_t epr_capable : 1;
        uint32_t unchunked_ext_msg_supported : 1;
        uint32_t no_usb_suspend : 1;
        uint32_t usb_comm_capable : 1;
        uint32_t capability_mismatch : 1;
        uint32_t : 1; // "giveback" flag deprecated
        uint32_t obj_position : 4; // !!! numeration starts from 1 !!!
    };
    uint32_t raw_value;
};

// [rev3.2] Table 6.25 PPS Request Data Object
union RDO_PPS {
    struct {
        uint32_t operating_current : 7; // 50ma step
        uint32_t : 2;
        uint32_t output_voltage : 12; // 20mv step
        uint32_t : 1;
        uint32_t epr_capable : 1;
        uint32_t unchunked_ext_msg_supported : 1;
        uint32_t no_usb_suspend : 1;
        uint32_t usb_comm_capable : 1;
        uint32_t capability_mismatch : 1;
        uint32_t : 1; // "giveback" flag deprecated
        uint32_t obj_position : 4; // !!! numeration starts from 1 !!!
    };
    uint32_t raw_value;
};

// [rev3.2] Table 6.26 AVS Request Data Object
// Skipped for now. Exactly the same as RDO_PPS in spec 3.2

// Helper to parse flags
union RDO_ANY {
    struct {
        uint32_t : 22; // depends on PDO type
        uint32_t epr_capable : 1;
        uint32_t unchunked_ext_msg_supported : 1;
        uint32_t no_usb_suspend : 1;
        uint32_t usb_comm_capable : 1;
        uint32_t capability_mismatch : 1;
        uint32_t : 1; // "giveback" flag deprecated
        uint32_t obj_position : 4; // !!! numeration starts from 1 !!!
    };
    uint32_t raw_value;
};

//
// SNK PDO objects, to describe Sink demands
//

// [rev3.2] 6.4.1.3.1 Sink Fixed Supply Power Data Objec
// Table 6.17
union SNK_PDO_FIXED {
    struct {
        uint32_t max_current : 10; // 10ma step
        uint32_t voltage : 10; // 50mv step
        uint32_t : 2;
        uint32_t frs_required : 2;
        uint32_t dual_role_data : 1;
        uint32_t usb_comms_capable : 1;
        uint32_t unconstrained_power : 1;
        uint32_t higher_capability : 1;
        uint32_t dual_role_power : 1;
        uint32_t pdo_type : 2;
    };
    uint32_t raw_value;
};

// [rev3.2] 6.4.1.3.4.1 SPR Programmable Power Supply APDO
// Table 6.20
union SNK_PDO_SPR_PPS {
    struct {
        uint32_t max_current: 7; // 50ma step
        uint32_t : 1;
        uint32_t min_voltage : 8; // 100mv step
        uint32_t : 1;
        uint32_t max_voltage : 8; // 100mv step
        uint32_t : 3;
        uint32_t apdo_subtype : 2; // 00b
        uint32_t pdo_type : 2; // 11b
    };
    uint32_t raw_value;
};

// [rev3.2] 6.4.1.3.4.3 EPR Adjustable Voltage Supply APDO
// Table 6.22
union SNK_PDO_EPR_AVS {
    struct {
        uint32_t pdp: 8; // 1W step
        uint32_t min_voltage : 8; // 100mv step
        uint32_t : 1;
        uint32_t max_voltage : 9; // 100mv step
        uint32_t : 2;
        uint32_t apdo_subtype : 2; // 01b
        uint32_t pdo_type : 2; // 11b
    };
    uint32_t raw_value;
};

namespace DO_TOOLS {
    inline bool is_fixed(uint32_t pdo) {
        PDO_FIXED pd = { .raw_value = pdo };
        return pd.pdo_type == PDO_TYPE::FIXED;
    }

    inline bool is_spr_pps(uint32_t pdo) {
        PDO_SPR_PPS pd = { .raw_value = pdo };
        return pd.pdo_type == PDO_TYPE::AUGMENTED &&
            pd.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_PPS;
    }
}; // namespace DO_TOOLS

namespace EPR_MODE_ACTION {
    enum Type {
        ENTER = 1,
        ENTER_ACKNOWLEDGED = 2,
        ENTER_SUCCEEDED = 3,
        ENTER_FAILED = 4,
        EXIT = 5
    };
}; // namespace ERP_MODE_ACTION

// [rev3.2] Table 6.50 EPR Mode Data Object (EPRMDO)
union EPRMDO {
    struct {
        uint32_t : 16;
        uint32_t data : 8;
        uint32_t action : 8;
    };
    uint32_t raw_value;
};

// [rev3.2] 6.5.14 Extended_Control Message
union ECDB {
    struct {
        uint16_t type : 8;
        uint16_t data : 8;
    };
    uint16_t raw_value;
};

// [rev3.2] 6.5.14 Extended_Control Message
namespace PD_EXT_CTRL_MSGT {
    enum type {
        EPR_Get_Source_Cap = 1,
        EPR_Get_Sink_Cap = 2,
        EPR_KeepAlive = 3,
        EPR_KeepAlive_Ack = 4
    };
}; // namespace PD_EXT_CTRL_MSGT

// [rev3.2] 6.4.12 Revision Message
// Table 6.52 Revision Message Data Object (RMDO)
union RMDO {
    struct {
        uint32_t : 16;
        uint32_t ver_minor : 4;
        uint32_t ver_major : 4;
        uint32_t rev_minor : 4;
        uint32_t rev_major : 4;
    };
    uint32_t raw_value;
};

// [rev3.2] 6.4.3 BIST Message
namespace BIST_MODE {
    enum Type {
        Carrier = 5,
        TestData = 8,
        SharedCapacityEnter = 9,
        SharedCapacityExit = 10,
    };
}; // namespace BIST_MODE

union BISTDO {
    struct {
        uint32_t : 28;
        uint32_t mode : 4;
    };
    uint32_t raw_value;
};


class I_PD_MSG {
public:
    PD_HEADER header;
    etl::ivector<uint8_t>& data;

    I_PD_MSG(etl::ivector<uint8_t>& data_ref) : data(data_ref) {}

    I_PD_MSG& operator=(const I_PD_MSG& other) {
        if (this != &other) {
            header = other.header;
            data.assign(other.data.begin(), other.data.end());
        }
        return *this;
    }

    virtual bool is_data_msg(uint8_t type) const = 0;
    virtual bool is_ctrl_msg(uint8_t type) const = 0;
    virtual bool is_ext_msg(uint8_t type) const = 0;
    virtual bool is_ext_ctrl_msg(uint8_t type) const = 0;

    virtual uint16_t read16(size_t pos) const = 0;
    virtual uint32_t read32(size_t pos) const = 0;
    virtual void append16(uint16_t value) = 0;
    virtual void append32(uint32_t value) = 0;

    virtual ~I_PD_MSG() = default;
};

template <int BufferSize>
struct PD_MSG_TPL : public I_PD_MSG {
    etl::vector<uint8_t, BufferSize> _buffer;

    PD_MSG_TPL() : I_PD_MSG(_buffer) {}

    bool is_data_msg(uint8_t type) const override{
        return header.extended == 0 && header.data_obj_count > 0 &&
            header.message_type == type;
    }
    bool is_ctrl_msg(uint8_t type) const override {
        return header.extended == 0 && header.data_obj_count == 0 &&
            header.message_type == type;
    }
    bool is_ext_msg(uint8_t type) const override {
        return header.extended > 0 && header.message_type == type;
    }
    bool is_ext_ctrl_msg(uint8_t type) const override {
        if (!is_ext_msg(PD_EXT_MSGT::Extended_Control) || data.size() < 2) {
            return false;
        }
        ECDB ecdb{.raw_value = read16(0)};
        return ecdb.type == type;
    }

    // Helpers to simplify payload access
    uint16_t read16(size_t pos) const override {
        return uint16_t(data[pos]) | (uint16_t(data[pos + 1]) << 8);
    }
    uint32_t read32(size_t pos) const override {
        return uint32_t(data[pos]) |
            (uint32_t(data[pos + 1]) << 8) |
            (uint32_t(data[pos + 2]) << 16) |
            (uint32_t(data[pos + 3]) << 24);
    }

    void append16(uint16_t value) override {
        data.push_back(value & 0xff);
        data.push_back((value >> 8) & 0xff);
    }
    void append32(uint32_t value) override {
        data.push_back(value & 0xff);
        data.push_back((value >> 8) & 0xff);
        data.push_back((value >> 16) & 0xff);
        data.push_back((value >> 24) & 0xff);
    }
};

using PD_MSG = PD_MSG_TPL<MaxExtendedMsgLen>;

} // namespace pd
