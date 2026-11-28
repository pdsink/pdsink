#include "harness/tester.h"

using namespace cts;
using namespace pd;

CTS_TEST("TEST.PD.VDM.SNK", "7_Unrecognized_VID_in_Unstructured_VDM", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2: Unstructured VDM Header (B15 = 0), VID of the Tester manufacturer
    constexpr uint32_t tester_vid = 0x1A0A; // CTS 1.4.10
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Vendor_Defined, { tester_vid << 16 }));

    // Step 3
    if (t.is_pd2()) {
        EXPECT_FALSE(t.receive(tReceiverResponse.max)); // [#1] Ignore
    } else {
        EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Not_Supported, tReceiverResponse)); // [#1]
    }
}
