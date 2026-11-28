#include "harness/tester.h"

using namespace cts;
using namespace pd;

CTS_TEST("TEST.PD.PROT.SNK", "1_Get_Sink_Cap_Response", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    if (!t.is_pd2()) { t.set_rp(Rp::SinkTxNG); }
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));

    // Step 3
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse)); // [#1]
}

CTS_TEST("TEST.PD.PROT.SNK", "3_SinkWaitCapTimer_Deadline", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    const auto vbus_on = t.hard_reset_cycle_vbus();

    // Step 4: SinkTxNG is presented while waiting for a response in PD3
    if (!t.is_pd2()) { t.set_rp(Rp::SinkTxNG); }

    // Step 3: Source_Capabilities immediately prior to tTypeCSinkWaitCap min
    t.wait(tTypeCSinkWaitCap.min - 1 * ms);
    bool acked = t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma });

    // Step 5: resend until tTypeCSinkWaitCap max or GoodCRC
    while (!acked && t.now() + tTypeCSendSourceCap.min <= vbus_on + tTypeCSinkWaitCap.max) {
        t.wait(tTypeCSendSourceCap.min);
        acked = t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma });
    }
    ASSERT_TRUE(acked);

    // Steps 4, 6: Request (Hard Reset instead fails the expectation)
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#1], [#2]
}

CTS_TEST("TEST.PD.PROT.SNK", "4_SinkWaitCapTimer_Timeout", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    const auto vbus_on = t.hard_reset_cycle_vbus();

    // Steps 3-4: no Source_Capabilities.
    // CTS: this check is a warning. Our stack must do it, so treat as a failure.
    EXPECT_TRUE(t.expect_hard_reset(tTypeCSinkWaitCap, vbus_on)); // [#1]
}

CTS_TEST("TEST.PD.PROT.SNK", "5_SenderResponseTimer_Deadline", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    if (!t.is_pd2()) { t.set_rp(Rp::SinkTxNG); }
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));

    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse);
    ASSERT_TRUE(request);

    // Step 4
    t.wait(t.is_pd2() ? tCtsSrcAcceptPD2.max : tCtsSrcAcceptPD3.max);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));

    // Step 5. CTS checks only the absence of a Hard Reset, but any message here
    // is unexpected in Ready [COMMON.CHECK.PD.5#1]
    EXPECT_FALSE(t.receive(tCtsSenderResponse.max, request->time)); // [#1]
}

CTS_TEST("TEST.PD.PROT.SNK", "6_SenderResponseTimer_Timeout", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    if (!t.is_pd2()) { t.set_rp(Rp::SinkTxNG); }
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse));

    // Steps 4-5: no Accept
    const auto tSenderResponse = t.is_pd2() ? tSenderResponsePD2 : tSenderResponsePD3;
    EXPECT_TRUE(t.expect_hard_reset(tSenderResponse.add_max(tSendHardReset))); // [#1]
}

CTS_TEST("TEST.PD.PROT.SNK", "7_PSTransitionTimer_Timeout", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    if (!t.is_pd2()) { t.set_rp(Rp::SinkTxNG); }
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#1]

    // Steps 4-6: no PS_RDY
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));
    EXPECT_TRUE(t.expect_hard_reset(tPSTransition)); // [#2]
}

CTS_TEST("TEST.PD.PROT.SNK", "8_Atomic_Message_Sequence_–_Accept", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    t.hard_reset_cycle_vbus();

    // Steps 3-4
    t.wait(tTypeCSinkWaitCap.min - 1 * ms);
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#1]

    // Steps 5-6: Get_Sink_Cap instead of Accept
    t.wait(tReceiverResponse.max);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    ASSERT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tProtErrSoftReset)); // [#2]

    // Step 7
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));

    // Step 8
    ASSERT_TRUE(t.send_source_caps_until_acked({ pdos::fixed_5v_500ma }));

    // Step 9
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#3]
}

CTS_TEST("TEST.PD.PROT.SNK", "9_Atomic_Message_Sequence_–_PS_RDY", REV_ALL) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    t.hard_reset_cycle_vbus();
    t.wait(tTypeCSinkWaitCap.min - 1 * ms);
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, { pdos::fixed_5v_500ma }));

    // Step 4
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#1]

    // Steps 5-6: Get_Sink_Cap instead of PS_RDY
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Accept));
    t.wait(tSrcTransReq.max);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));

    // Step 7
    EXPECT_TRUE(t.expect_hard_reset(tCtsHardReset)); // [#2]
}

CTS_TEST("TEST.PD.PROT.SNK", "14_Valid_Use_of_GoodCRC_on_Power_up", REV_ALL) {
    // Steps 1-3
    t.set_rp(Rp::Default);
    t.set_vbus(true);

    // Step 4
    t.wait(1 * ms);

    // Steps 5-6
    ASSERT_TRUE(t.send_source_caps_until_acked({
        pdos::fixed_5v_500ma | pdos::unconstrained_power
    }));

    // Step 7
    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#1]
    ASSERT_TRUE(request);

    // Steps 8-10
    ASSERT_TRUE(t.accept_request());
}
