#include "harness/tester.h"

using namespace cts;
using namespace pd;

// Only BIST Carrier Mode entry/duration is checked. Bit rate is PHY level.
CTS_TEST("TEST.PD.PHY.ALL", "1_Transmit_Bit_Rate_and_the_Drift", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    BISTDO bdo{};
    bdo.mode = BIST_MODE::Carrier;
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::BIST, { bdo.raw_value }));

    // Step 9
    auto start = t.expect_bist_mode(TCPC_BIST_MODE::Carrier, tBISTCarrierMode);
    ASSERT_TRUE(start);
    EXPECT_TRUE(t.expect_bist_mode(TCPC_BIST_MODE::Off, tBISTContMode, *start)); // [#3]
}
