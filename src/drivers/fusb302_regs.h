#pragma once

#include <stdint.h>

namespace pd {

namespace fusb302 {

union DeviceID {
    struct {
        uint8_t REVISION_ID : 2;
        uint8_t PRODUCT_ID : 2;
        uint8_t VERSION_ID : 4;
    };
    uint8_t raw_value;
    enum { addr = 0x01 };
};

union Switches0 {
    struct {
        uint8_t PDWN1 : 1;
        uint8_t PDWN2 : 1;
        uint8_t MEAS_CC1 : 1;
        uint8_t MEAS_CC2 : 1;
        uint8_t VCONN_CC1 : 1;
        uint8_t VCONN_CC2 : 1;
        uint8_t PU_EN1 : 1;
        uint8_t PU_EN2 : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x02 };
};

union Switches1 {
    struct {
        uint8_t TXCC1 : 1;
        uint8_t TXCC2 : 1;
        uint8_t AUTO_CRC : 1;
        uint8_t : 1;
        uint8_t DATAROLE : 1;
        uint8_t SPECREV : 2;
        uint8_t POWERROLE : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x03 };
};

union Measure {
    struct {
        uint8_t MDAC : 6;
        uint8_t MEAS_VBUS : 1;
        uint8_t : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x04 };
};

union Slice {
    struct {
        uint8_t SDAC : 6;
        uint8_t SDAC_HYS : 2;
    };
    uint8_t raw_value;
    enum { addr = 0x05 };
};

union Control0 {
    struct {
        uint8_t TX_START : 1;
        uint8_t AUTO_PRE : 1;
        uint8_t HOST_CUR : 2;
        uint8_t : 1;
        uint8_t INT_MASK : 1;
        uint8_t TX_FLUSH : 1;
        uint8_t : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x06 };
};

union Control1 {
    struct {
        uint8_t ENSOP1 : 1;
        uint8_t ENSOP2 : 1;
        uint8_t RX_FLUSH : 1;
        uint8_t : 1;
        uint8_t BIST_MODE2 : 1;
        uint8_t ENSOP1DB : 1;
        uint8_t ENSOP2DB : 1;
        uint8_t : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x07 };
};

union Control2 {
    struct {
        uint8_t TOGGLE : 1;
        uint8_t MODE : 2;
        uint8_t WAKE_EN : 1;
        uint8_t : 1;
        uint8_t TOG_RD_ONLY : 1;
        uint8_t TOG_SAVE_PWR : 2;
    };
    uint8_t raw_value;
    enum { addr = 0x08 };
};

union Control3 {
    struct {
        uint8_t AUTO_RETRY : 1;
        uint8_t N_RETRIES : 2;
        uint8_t AUTO_SOFTRESET : 1;
        uint8_t AUTO_HARDRESET : 1;
        uint8_t BIST_TMODE : 1;
        uint8_t SEND_HARD_RESET : 1;
        uint8_t : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x09 };
};

union Mask1 {
    struct {
        uint8_t M_BC_LVL : 1;
        uint8_t M_COLLISION : 1;
        uint8_t M_WAKE : 1;
        uint8_t M_ALERT : 1;
        uint8_t M_CRC_CHK : 1;
        uint8_t M_COMP_CHNG : 1;
        uint8_t M_ACTIVITY : 1;
        uint8_t M_VBUSOK : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x0A };
};

union Power {
    struct {
        uint8_t PWR : 4;
        uint8_t : 4;
    };
    uint8_t raw_value;
    enum { addr = 0x0B };
};

union Reset {
    struct {
        uint8_t PD_RESET : 1;
        uint8_t SW_RES : 1;
        uint8_t : 6;
    };
    uint8_t raw_value;
    enum { addr = 0x0C };
};

union OCPreg {
    struct {
        uint8_t OCP_CUR : 3;
        uint8_t OCP_RANGE : 1;
        uint8_t : 4;
    };
    uint8_t raw_value;
    enum { addr = 0x0D };
};

union Maska {
    struct {
        uint8_t M_HARDRST : 1;
        uint8_t M_SOFTRST : 1;
        uint8_t M_TXSENT : 1;
        uint8_t M_HARDSENT : 1;
        uint8_t M_RETRYFAIL : 1;
        uint8_t M_SOFTFAIL : 1;
        uint8_t M_TOGDONE : 1;
        uint8_t M_OCP_TEMP : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x0E };
};

union Maskb {
    struct {
        uint8_t M_GCRCSENT : 1;
        uint8_t : 7;
    };
    uint8_t raw_value;
    enum { addr = 0x0F };
};

union Control4 {
    struct {
        uint8_t TOG_EXIT_AUD : 1;
        uint8_t : 7;
    };
    uint8_t raw_value;
    enum { addr = 0x10 };
};

union Status0a {
    struct {
        uint8_t HARDRST : 1;
        uint8_t SOFTRST : 1;
        uint8_t POWER23 : 2;
        uint8_t RETRYFAIL : 1;
        uint8_t SOFTFAIL : 1;
        uint8_t : 2;
    };
    uint8_t raw_value;
    enum { addr = 0x3C };
};

union Status1a {
    struct {
        uint8_t RXSOP : 1;
        uint8_t RXSOP1DB : 1;
        uint8_t RXSOP2DB : 1;
        uint8_t TOGSS : 3;
        uint8_t : 2;
    };
    uint8_t raw_value;
    enum { addr = 0x3D };
};

union Interrupta {
    struct {
        uint8_t I_HARDRST : 1;
        uint8_t I_SOFTRST : 1;
        uint8_t I_TXSENT : 1;
        uint8_t I_HARDSENT : 1;
        uint8_t I_RETRYFAIL : 1;
        uint8_t I_SOFTFAIL : 1;
        uint8_t I_TOGDONE : 1;
        uint8_t I_OCP_TEMP : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x3E };
};

union Interruptb {
    struct {
        uint8_t I_GCRCSENT : 1;
        uint8_t : 7;
    };
    uint8_t raw_value;
    enum { addr = 0x3F };
};

union Status0 {
    struct {
        uint8_t BC_LVL : 2;
        uint8_t WAKE : 1;
        uint8_t ALERT : 1;
        uint8_t CRC_CHK : 1;
        uint8_t COMP : 1;
        uint8_t ACTIVITY : 1;
        uint8_t VBUSOK : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x40 };
};

union Status1 {
    struct {
        uint8_t OCP : 1;
        uint8_t OVRTEMP : 1;
        uint8_t TX_FULL : 1;
        uint8_t TX_EMPTY : 1;
        uint8_t RX_FULL : 1;
        uint8_t RX_EMPTY : 1;
        uint8_t RXSOP1 : 1;
        uint8_t RXSOP2 : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x41 };
};

union Interrupt {
    struct {
        uint8_t I_BC_LVL : 1;
        uint8_t I_COLLISION : 1;
        uint8_t I_WAKE : 1;
        uint8_t I_ALERT : 1;
        uint8_t I_CRC_CHK : 1;
        uint8_t I_COMP_CHNG : 1;
        uint8_t I_ACTIVITY : 1;
        uint8_t I_VBUSOK : 1;
    };
    uint8_t raw_value;
    enum { addr = 0x42 };
};

namespace FIFOs {
    enum { addr = 0x43 };
};

} // namespace fusb302

} // namespace pd