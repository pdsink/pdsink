#pragma once

#include <stdint.h>

namespace pd {

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

} // namespace pd
