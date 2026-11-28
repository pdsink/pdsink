#include "harness/tester.h"

using namespace cts;
using namespace pd;

// With hardware GoodCRC check, TCPC ignores GoodCRC with incorrect CRC or
// MessageID, same as no GoodCRC at all.
CTS_TEST("TEST.PD.PROT.ALL", "1_Corrupted_GoodCRC", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-4: incorrect CRC
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    auto caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#2]
    ASSERT_TRUE(caps);

    // Step 5: retry is acknowledged
    t.set_goodcrc_reply(true);
    ASSERT_TRUE(t.expect_retries(*caps, 1, tReceive.add_max(tRetry))); // [#5]

    // Steps 6-7. Step 8 applies to the reply, so GoodCRC is disabled before.
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#7]
    ASSERT_TRUE(caps);

    // Steps 8-10: incorrect MessageID, nRetryCount times
    ASSERT_TRUE(t.expect_retries(*caps, t.nRetryCount(), tReceive.add_max(tRetry))); // [#10]

    // Not in CTS. Transmission failed, so UUT sends Soft_Reset. The Tester acts
    // as an actual port (CTS 1.4.6): accepts it and re-negotiates the contract.
    t.set_goodcrc_reply(true);
    ASSERT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tReceive.add_max(tSoftReset)));
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));
    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse);
    ASSERT_TRUE(request);
    ASSERT_TRUE(t.accept_request()); // COMMON.PROC.PD.10

    // Steps 12-14: incorrect CRC
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse);
    ASSERT_TRUE(caps);

    // Step 15: new AMS before tRetry expires
    t.wait(35 * us);
    t.set_goodcrc_reply(true);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Source_Cap));

    // Step 16
    auto frame = t.receive(25 * ms);
    const bool retried = frame && frame->same_msg(*caps);
    EXPECT_FALSE(retried); // [#11]

    // Step 17
    if (!frame || retried) {
        EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tProtErrSoftReset)); // [#12]
    } else {
        EXPECT_TRUE(frame->is_ctrl(PD_CTRL_MSGT::Soft_Reset)); // [#12]
    }
}

CTS_TEST("TEST.PD.PROT.ALL", "2_Soft_Reset_and_Hard_Reset", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    //
    // Steps 2-5: UUT reply is not acknowledged => retries
    //
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));

    auto caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#2]
    ASSERT_TRUE(caps);
    ASSERT_TRUE(t.expect_retries(*caps, t.nRetryCount(), tReceive.add_max(tRetry))); // [#5]

    //
    // Step 6b: Soft Reset, then recover contract
    //
    t.set_goodcrc_reply(true);
    ASSERT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tReceive.add_max(tSoftReset))); // [#7]

    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));

    auto req = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#8]
    ASSERT_TRUE(req);
    ASSERT_TRUE(t.accept_request()); // COMMON.PROC.PD.10: Accept + PS_RDY

    //
    // Steps 7-10: second AMS, reply not acknowledged again
    //
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));

    caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#12]
    ASSERT_TRUE(caps);
    ASSERT_TRUE(t.expect_retries(*caps, t.nRetryCount(), tReceive.add_max(tRetry))); // [#15]

    //
    // Step 11: Soft Reset not acknowledged => retries => Hard Reset
    //
    auto sr = t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tReceive.add_max(tSoftReset)); // [#17]
    ASSERT_TRUE(sr);

    ASSERT_TRUE(t.expect_retries(*sr, t.nRetryCount(), tReceive.add_max(tRetry))); // [#18]
    EXPECT_TRUE(t.expect_hard_reset(tHardReset.add_max(tReceive))); // [#19]
}

CTS_TEST("TEST.PD.PROT.ALL", "3_Soft_Reset_Response", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-4: until MessageID of UUT reply is 000b
    for (int i = 0; i < 8; i++) {
        ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
        auto caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#2]
        ASSERT_TRUE(caps);
        if (caps->chunk.header.message_id == 0) { break; }
    }

    // Step 5
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Soft_Reset));
    auto accept = t.expect_ctrl_msg(PD_CTRL_MSGT::Accept, tReceiverResponse); // [#4]
    ASSERT_TRUE(accept);
    EXPECT_EQ(accept->chunk.header.message_id, 0);

    // Step 6a
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));
    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#5]
    ASSERT_TRUE(request);
    ASSERT_TRUE(t.accept_request()); // COMMON.PROC.PD.10

    // Steps 7-8
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse)); // [#8]
}

CTS_TEST("TEST.PD.PROT.ALL", "4_Reset_Signals_and_MessageID", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-4: until MessageID of UUT reply is 000b
    for (int i = 0; i < 8; i++) {
        ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
        auto caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse); // [#2]
        ASSERT_TRUE(caps);
        if (caps->chunk.header.message_id == 0) { break; }
    }

    // Steps 5-6a
    t.hard_reset_cycle_vbus();

    ASSERT_TRUE(t.send_source_caps_until_acked({ pdos::fixed_5v_500ma }));

    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#4]
    ASSERT_TRUE(request);
    ASSERT_TRUE(t.accept_request()); // COMMON.PROC.PD.10
    t.wait_for_frame(tCtsAfterLastPoint.max);

    // Steps 7-8
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse)); // [#9]

    // Steps 9-10: same MessageID must be ignored
    ASSERT_TRUE(t.resend_last_msg());
    EXPECT_FALSE(t.receive(tCtsSenderResponse.max)); // [#11]

    // Steps 11-12
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse)); // [#13]
}

CTS_TEST("TEST.PD.PROT.ALL", "5_Unrecognized_Message", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2: Reserved Control Message type
    if (t.is_pd2()) {
        ASSERT_TRUE(t.send_ctrl_msg(static_cast<PD_CTRL_MSGT::Type>(0b1110)));
        EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Reject, tReceiverResponse)); // [#2]
    } else {
        ASSERT_TRUE(t.send_ctrl_msg(static_cast<PD_CTRL_MSGT::Type>(0b11111)));
        EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Not_Supported, tReceiverResponse)); // [#2]
    }
}
