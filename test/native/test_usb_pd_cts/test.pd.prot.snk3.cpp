#include "harness/tester.h"

using namespace cts;
using namespace pd;

CTS_TEST("TEST.PD.PROT.SNK3", "2_Alert_Response_Source_Input_Change", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    PD_ALERT ado{};
    ado.otp = 1;
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Alert, { ado.raw_value }));
    t.set_rp(Rp::SinkTxOK);

    // Step 4: Get_Status or no message. Not_Supported is not allowed,
    // because UUT has sink PPS APDO.
    auto response = t.receive(tSenderResponsePD3.min);
    if (response) {
        EXPECT_TRUE(response->is_ctrl(PD_CTRL_MSGT::Get_Status)); // [#1]
    }
}

CTS_TEST("TEST.PD.PROT.SNK3", "4_Soft_Reset_Sent_Regardless_of_Rp_Value", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    t.set_rp(Rp::SinkTxNG);
    t.set_goodcrc_reply(false);
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));

    auto caps = t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse);
    ASSERT_TRUE(caps);
    ASSERT_TRUE(t.expect_retries(*caps, t.nRetryCount(), tReceive.add_max(tRetry)));

    // Step 4
    t.set_goodcrc_reply(true);
    EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tSoftReset.add_max(tReceive))); // [#1]
}

CTS_TEST("TEST.PD.PROT.SNK3", "5_Sink_PPS_Normal_Operation", REV3) {
    // UUT policy: request the PPS APDO
    t.dpm.trigger_variant(PDO_VARIANT::APDO_PPS, 5000);

    // Step 1
    auto request = t.bring_up_sink_pps(); // COMMON.PROC.BU.5
    ASSERT_TRUE(request);

    // Step 2
    t.set_rp(Rp::SinkTxOK);

    // Step 3
    request = t.expect_data_msg(PD_DATA_MSGT::Request, tPPSRequest, request->time); // [#1]
    ASSERT_TRUE(request);

    // Step 4a: CTS checks position 2, but BU.5 has the PPS APDO at 5 (see README Notes)
    EXPECT_EQ(RDO_ANY{request->chunk.read32(0)}.obj_position, 5u); // [#2]

    // Step 5
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait));
    const auto wait_sent = t.now();
    // Request exactly at tSinkRequest min is allowed
    EXPECT_FALSE(t.receive(tSinkRequest.min - 1 * us)); // [#3]

    // Step 6: reply Wait to each Request. Disconnect (Rd release) is not
    // modeled by the driver mock.
    auto previous = *request;
    while (auto frame = t.receive(tPPSTimeout.max, wait_sent)) {
        ASSERT_TRUE(frame->is_data(PD_DATA_MSGT::Request)); // [#4]
        previous = *frame;
        ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait));
    }

    // Step 7
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tPPSRequest, previous.time)); // [#5]
}

CTS_TEST("TEST.PD.PROT.SNK3", "6_Revision_Number_Test", REV3) {
    // Steps 1-3
    t.set_rp(Rp::Default);
    t.set_vbus(true);

    // Step 4
    t.wait(50 * ms);

    // Steps 5-6
    t.set_tx_spec_revision(0b11);
    ASSERT_TRUE(t.send_source_caps_until_acked({
        pdos::fixed_5v_500ma | pdos::unconstrained_power
    }));

    // Step 7
    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#1]
    ASSERT_TRUE(request);
    EXPECT_EQ(request->chunk.header.spec_revision, t.revision());
}

CTS_TEST("TEST.PD.PROT.SNK3", "9_Alert_Response_Extended_Alert", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    PD_ALERT ado{};
    ado.extended = 1;
    ado.ext_type = 1; // Power state change
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Alert, { ado.raw_value }));

    // Step 3
    t.wait(tSrcHoldsBus.max);
    t.set_rp(Rp::SinkTxOK);

    // Step 4
    auto response = t.receive(tSenderResponsePD3.max);
    if (response) {
        // Anything else is an unexpected message in Ready [COMMON.CHECK.PD.5#4]
        ASSERT_TRUE(response->is_ctrl(PD_CTRL_MSGT::Not_Supported) ||
            response->is_ctrl(PD_CTRL_MSGT::Get_Status));
        return;
    }

    // Step 5
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, {
        pdos::fixed_5v_500ma | pdos::dual_role_power | pdos::unconstrained_power
    }));

    // Step 6
    EXPECT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse)); // [#1]
}
