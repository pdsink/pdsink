#include "harness/tester.h"

using namespace cts;
using namespace pd;

// Invalid reset signals (steps 4-7) are PHY level, filtered by TCPC.
// Only "messages ignored in BIST Test Data mode" is checked.
CTS_TEST("TEST.PD.PHY.PORT", "1_Invalid_Reset_Signals", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    BISTDO bdo{};
    bdo.mode = BIST_MODE::TestData;
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::BIST, { bdo.raw_value }));
    t.wait(tCtsBIST.max);

    // Steps 8-9. In BIST Test Data mode the UUT ignores messages by design,
    // so COMMON.CHECK.PD.4 is not applicable.
    t.set_response_check(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    EXPECT_FALSE(t.receive(tCtsSenderResponse.max)); // [#2]
}
