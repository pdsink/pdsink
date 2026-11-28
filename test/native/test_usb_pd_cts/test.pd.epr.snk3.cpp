#include "harness/tester.h"

using namespace cts;
using namespace pd;

namespace {

// CTS 7.2: Source_Capabilities with 4 PDOs, EPR Mode Capable set
auto spr_source_caps() -> std::vector<uint32_t> {
    return {
        pdos::fixed_5v_3a | pdos::dual_role_power |
            pdos::unconstrained_power | pdos::epr_capable,
        pdos::fixed_9v_3a, pdos::fixed_15v_3a, pdos::fixed_20v_5a
    };
}

// COMMON.PROC.PD3.3 step 2.b: 7 SPR PDOs + 4 EPR PDOs (240W).
// Without EPR PDOs, only the SPR part is sent.
auto epr_source_caps(bool with_epr_pdos = true) -> std::vector<uint32_t> {
    auto caps = spr_source_caps();

    caps.push_back(pdos::spr_avs_15v_3a_20v_5a);
    caps.push_back(0);
    caps.push_back(0);

    if (with_epr_pdos) {
        caps.push_back(pdos::fixed_28v_5a);
        caps.push_back(pdos::fixed_36v_5a);
        caps.push_back(pdos::fixed_48v_5a);

        caps.push_back(pdos::epr_avs_15_48v_240w);
    }
    return caps;
}

bool send_epr_mode(Tester& t, EPR_MODE_ACTION::Type action, uint8_t data = 0) {
    EPRMDO mdo{};
    mdo.action = action;
    mdo.data = data;
    return t.send_data_msg(PD_DATA_MSGT::EPR_Mode, { mdo.raw_value });
}

// Steps 2-3 of most EPR.SNK3 tests. Returns EPR_Mode (Enter) from UUT,
// or nullopt when the test ends (failure is reported if any).
auto expect_epr_mode_enter(Tester& t, const Frame& request) -> std::optional<Frame> {
    if (!RDO_ANY{request.chunk.read32(0)}.epr_capable) {
        ADD_FAILURE() << "EPR Mode Capable bit is not set in Request";
        return std::nullopt;
    }

    // CTS: no EPR_Mode (Enter) stops the test with a warning. Our stack always
    // enters EPR Mode, so treat it as a failure.
    auto enter = t.receive(5000 * ms);
    if (!enter) {
        ADD_FAILURE() << "No EPR_Mode (Enter) from UUT";
        return std::nullopt;
    }

    if (!enter->is_epr_mode(EPR_MODE_ACTION::ENTER)) {
        ADD_FAILURE() << "Expected EPR_Mode (Enter)";
        return std::nullopt;
    }
    return enter;
}

// COMMON.PROC.PD3.4 steps 2-3: Accept -> PS_RDY
bool accept_epr_request(Tester& t) {
    if (!t.send_ctrl_msg(PD_CTRL_MSGT::Accept)) { return false; }
    t.wait(tSrcTransReqEPR.max);
    return t.send_ctrl_msg(PD_CTRL_MSGT::PS_RDY);
}

// EPR_Source_Capabilities -> EPR_Request -> Accept -> PS_RDY
// (COMMON.PROC.PD3.3 step 2.b, COMMON.PROC.PD3.4)
auto establish_epr_contract(Tester& t, const std::vector<uint32_t>& caps) -> std::optional<Frame> {
    if (!t.send_epr_source_caps(caps)) { return std::nullopt; }

    auto request = t.expect_data_msg(PD_DATA_MSGT::EPR_Request, tReceiverResponse);
    if (!request) { return std::nullopt; }

    if (!accept_epr_request(t)) { return std::nullopt; }
    return request;
}

// Watch UUT keep-alives for `duration`, acknowledging each one.
//
// NOTE: CTS allows any UUT-initiated message here, and treats a missing
// EPR_KeepAlive as a warning. This tester is written for this stack only, so
// the expected sequence is hardcoded. Revisit if it ever gets in the way.
bool watch_keep_alive(Tester& t, uint32_t duration) {
    const auto failures_before = test_failure_count();
    const auto end = t.now() + duration;
    auto last_uut_time_us = t.now();

    while (t.now() < end) {
        auto keep_alive = t.expect_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Keep_Alive,
            {0, tSinkEPRKeepAlive.max}, last_uut_time_us);
        if (!keep_alive) { return false; }
        last_uut_time_us = keep_alive->time;
        if (!t.send_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Keep_Alive_Ack)) { return false; }
    }
    return test_failure_count() == failures_before;
}

auto is_epr_profile_position(const Frame& epr_request) -> bool {
    return RDO_ANY{epr_request.chunk.read32(0)}.obj_position >= 8;
}

} // namespace

CTS_TEST("TEST.PD.EPR.SNK3", "1_EPR_Entry_Process_-_Success", REV3) {
    // Step 1
    auto request = t.bring_up_sink(spr_source_caps()); // COMMON.PROC.BU.2
    ASSERT_TRUE(request);

    // Step 2 is moved after step 6. UUT sends EPR_Mode (Enter) right after the
    // SPR contract, and any other message during EPR Mode entry causes Soft Reset
    // ([rev3.2 v1.2] 9.2.26.3.2). So the Tester completes the entry first.

    // Step 3 [#2]
    auto enter = expect_epr_mode_enter(t, *request);
    if (!enter) { return; }

    // COMMON.PROC.PD3.6. Not referenced by CTS here, but UUT waits for Enter
    // Acknowledged before Enter Succeeded ([rev3.2 v1.2] 9.2.26.3.2.1).
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_ACKNOWLEDGED));

    // Step 4: Captive_Cable = No

    // Step 5
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_SUCCEEDED));

    // Step 6
    t.wait(50 * ms);
    ASSERT_TRUE(establish_epr_contract(t, epr_source_caps())); // [#10]

    // Step 2 (moved)
    ASSERT_TRUE(t.send_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Get_Sink_Cap));
    ASSERT_TRUE(t.expect_ext_msg(PD_EXT_MSGT::EPR_Sink_Capabilities, tReceiverResponse)); // [#11]

    // Steps 7-8
    ASSERT_TRUE(watch_keep_alive(t, 10000 * ms)); // [#4]

    // Step 9
    ASSERT_TRUE(t.send_epr_source_caps(epr_source_caps()));
    auto epr_request = t.expect_data_msg(PD_DATA_MSGT::EPR_Request, tReceiverResponse); // [#13]
    ASSERT_TRUE(epr_request);

    // Steps 10-11. Disconnect (Rd release) is not modeled by the driver mock.
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait));
    const auto monitor_end = t.now() + tSourceEPRKeepAlive.max;
    auto last_uut_time_us = epr_request->time;
    while (t.now() < monitor_end) {
        auto frame = t.receive(tSinkEPRKeepAlive.max, last_uut_time_us);
        ASSERT_TRUE(frame); // [#12]
        last_uut_time_us = frame->time;

        if (frame->is_data(PD_DATA_MSGT::EPR_Request)) {
            ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait)); // [#14]
        } else {
            ASSERT_NO_FATAL_FAILURE(
                check_ext_ctrl_msg(*frame, PD_EXT_CTRL_MSGT::EPR_Keep_Alive));
            ASSERT_TRUE(t.send_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Keep_Alive_Ack));
        }
    }

    // Step 12
    epr_request = t.receive(1000 * ms);
    if (!epr_request || !epr_request->is_data(PD_DATA_MSGT::EPR_Request) ||
        is_epr_profile_position(*epr_request))
    {
        // A request we are not going to accept is answered with Wait, as in the
        // previous steps. Sending new capabilities without a response would
        // break the AMS started by the UUT.
        if (epr_request && epr_request->is_data(PD_DATA_MSGT::EPR_Request)) {
            ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait));
        }

        ASSERT_TRUE(t.send_epr_source_caps(epr_source_caps(false)));
        epr_request = t.expect_data_msg(PD_DATA_MSGT::EPR_Request, tReceiverResponse); // [#6]
        ASSERT_TRUE(epr_request);
    }

    // Step 13
    ASSERT_TRUE(accept_epr_request(t)); // COMMON.PROC.PD3.4 [#8]

    // Step 14
    auto exit = t.receive(100 * ms);
    if (exit) {
        ASSERT_TRUE(exit->is_epr_mode(EPR_MODE_ACTION::EXIT));
    } else {
        ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::EXIT));
    }

    // Step 15
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, spr_source_caps()));
    request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#7]
    ASSERT_TRUE(request);

    // Step 16
    ASSERT_TRUE(t.accept_request()); // COMMON.PROC.PD.10 [#9]
}

CTS_TEST("TEST.PD.EPR.SNK3", "2_EPR_Entry_Failed_-_tEnterEPR_timeout", REV3) {
    // Step 1
    auto request = t.bring_up_sink(spr_source_caps()); // COMMON.PROC.BU.2
    ASSERT_TRUE(request);

    // Steps 2-3 [#1], [#2]
    auto enter = expect_epr_mode_enter(t, *request);
    if (!enter) { return; }

    // Step 4
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_ACKNOWLEDGED));

    // Steps 6-7: no Enter_Succeeded
    EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tEnterEPR, enter->time)); // [#4]
}

CTS_TEST("TEST.PD.EPR.SNK3", "3_EPR_Entry_Failed_by_the_EPR_Mode_Enter_failed_Message", REV3) {
    // Step 6: repeat for all Data values and 1 reserved value (6), starting with 3
    for (uint8_t data : { 3, 0, 1, 2, 4, 5, 6 }) {
        SCOPED_TRACE("Enter Failed Data = " + std::to_string(data));

        // Step 1. Each run starts disconnected.
        t.set_vbus(false);
        t.set_rp(TCPC_CC_LEVEL::NONE);
        t.wait(tCtsAfterLastPoint.max);

        auto request = t.bring_up_sink(spr_source_caps()); // COMMON.PROC.BU.2
        ASSERT_TRUE(request);

        // Steps 2-3 [#1], [#2]
        auto enter = expect_epr_mode_enter(t, *request);
        if (!enter) { return; }

        // Steps 4-5
        ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_FAILED, data));
        ASSERT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tProtErrSoftReset)); // [#3]
    }
}

namespace {

// Bring up SPR and complete EPR entry, sending Enter Succeeded at Enter + 100 ms.
bool enter_epr(Tester& t) {
    auto request = t.bring_up_sink(spr_source_caps());
    if (!request) { return false; }

    auto enter = expect_epr_mode_enter(t, *request);
    if (!enter) { return false; }

    if (!send_epr_mode(t, EPR_MODE_ACTION::ENTER_ACKNOWLEDGED)) { return false; }

    t.wait_until(enter->time + 100 * ms);
    return send_epr_mode(t, EPR_MODE_ACTION::ENTER_SUCCEEDED);
}

} // namespace

CTS_TEST("TEST.PD.EPR.SNK3", "4_EPR_Fail_due_to_tFirstSourceCap_timeout.", REV3) {
    // Steps 1-5: COMMON.PROC.BU.2, EPR entry (steps 2-3: [#1], [#2])
    ASSERT_TRUE(enter_epr(t));

    // Steps 6-7: no EPR_Source_Capabilities
    EXPECT_TRUE(t.expect_hard_reset({0, tTypeCSinkWaitCap.max})); // [#3]
}

CTS_TEST("TEST.PD.EPR.SNK3", "5_EPR_Exit_due_to_incorrect_EPR_Source_Cap", REV3) {
    // Steps 1-5: COMMON.PROC.BU.2, EPR entry (steps 2-3: [#1], [#2])
    ASSERT_TRUE(enter_epr(t));

    // Step 6: 28V at SPR position 5
    auto caps = spr_source_caps();
    caps.push_back(pdos::fixed_28v_5a);
    caps.push_back(0);
    caps.push_back(0);
    caps.push_back(pdos::fixed_28v_5a);
    ASSERT_TRUE(t.send_epr_source_caps(caps));

    // Step 7
    EXPECT_TRUE(t.expect_hard_reset(tProtErrHardReset)); // [#3]
}

CTS_TEST("TEST.PD.EPR.SNK3", "6_EPR_Exit_due_to_EPR_Mode_Exit_Message", REV3) {
    // UUT policy: the test needs a contract on an EPR PDO (step 8). Set before
    // the bring-up, so the UUT asks for it in its first EPR_Request.
    t.dpm.trigger_any(28000);

    // Steps 1-5: COMMON.PROC.BU.2, EPR entry (steps 2-3: [#1], [#2])
    ASSERT_TRUE(enter_epr(t));

    // Step 6
    t.wait(50 * ms);
    auto epr_request = establish_epr_contract(t, epr_source_caps()); // [#5]
    ASSERT_TRUE(epr_request);

    // Step 7
    ASSERT_TRUE(watch_keep_alive(t, 1000 * ms)); // [#3]

    // Step 8
    if (!is_epr_profile_position(*epr_request)) { return; }
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::EXIT));

    // Step 9
    EXPECT_TRUE(t.expect_hard_reset(tProtErrHardReset)); // [#4]
}

CTS_TEST("TEST.PD.EPR.SNK3", "8_EPR_Exit_due_to_Source_Cap", REV3) {
    // Steps 1-5: COMMON.PROC.BU.2, EPR entry (steps 2-3: [#1], [#2])
    ASSERT_TRUE(enter_epr(t));

    // Step 6
    t.wait(50 * ms);
    ASSERT_TRUE(establish_epr_contract(t, epr_source_caps()));

    // Step 7
    ASSERT_TRUE(watch_keep_alive(t, 1000 * ms)); // [#3]

    // Steps 8-9
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, spr_source_caps()));
    EXPECT_TRUE(t.expect_hard_reset(tProtErrHardReset)); // [#4]
}

CTS_TEST("TEST.PD.EPR.SNK3", "9_EPR_Entry_failed_due_to_SourceCap", REV3) {
    // Steps 1-5: COMMON.PROC.BU.2, EPR entry (steps 2-3: [#1], [#2])
    ASSERT_TRUE(enter_epr(t));

    // Steps 6-7
    t.wait(50 * ms);
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, spr_source_caps()));
    EXPECT_TRUE(t.expect_hard_reset(tProtErrHardReset)); // [#4]
}

CTS_TEST("TEST.PD.EPR.SNK3", "10_EPR_Exit_fail_due_to_SinkWaitCapTimer_timeout", REV3) {
    // Step 1
    auto request = t.bring_up_sink(spr_source_caps()); // COMMON.PROC.BU.2
    ASSERT_TRUE(request);

    // Steps 2-3 [#1], [#2]
    auto enter = expect_epr_mode_enter(t, *request);
    if (!enter) { return; }

    // Step 3: EPR Entry process, COMMON.PROC.PD3.6
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_ACKNOWLEDGED));
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::ENTER_SUCCEEDED));
    auto epr_request = establish_epr_contract(t, epr_source_caps());
    ASSERT_TRUE(epr_request);

    // Steps 4-5
    if (is_epr_profile_position(*epr_request)) {
        epr_request = establish_epr_contract(t, epr_source_caps(false)); // [#3], [#4]
        ASSERT_TRUE(epr_request);
    }

    // Step 6: EPR Mode with SPR PDO, no Source_Capabilities after exit
    ASSERT_FALSE(is_epr_profile_position(*epr_request));
    ASSERT_TRUE(send_epr_mode(t, EPR_MODE_ACTION::EXIT));
    EXPECT_TRUE(t.expect_hard_reset({0, tTypeCSinkWaitCap.max + tReceive.max})); // [#6]
}

namespace {

// EPR.SNK3.12 steps 1-8. Returns the first EPR_Request for the caller to answer.
// Enter Succeeded is sent immediately, unlike enter_epr().
auto start_epr_negotiation(Tester& t) -> std::optional<Frame> {
    // Step 1
    auto request = t.bring_up_sink(spr_source_caps()); // COMMON.PROC.BU.2
    if (!request) { return std::nullopt; }

    // Step 4 [#3]
    if (!expect_epr_mode_enter(t, *request)) { return std::nullopt; }

    // COMMON.PROC.PD3.6. Not referenced by CTS here, but UUT waits for Enter
    // Acknowledged before Enter Succeeded ([rev3.2 v1.2] 9.2.26.3.2.1).
    if (!send_epr_mode(t, EPR_MODE_ACTION::ENTER_ACKNOWLEDGED)) { return std::nullopt; }

    // Step 5: Captive_Cable = No

    // Step 6
    if (!send_epr_mode(t, EPR_MODE_ACTION::ENTER_SUCCEEDED)) { return std::nullopt; }

    // Step 7
    if (!t.send_epr_source_caps(epr_source_caps())) { return std::nullopt; }

    // Step 8 [#5]
    return t.expect_data_msg(PD_DATA_MSGT::EPR_Request, tReceiverResponse);
}

// EPR.SNK3.12 step 12: 7 SPR PDOs + 3 EPR PDOs with changed capabilities
auto epr_source_caps_changed() -> std::vector<uint32_t> {
    auto caps = epr_source_caps(false);
    caps.push_back(pdos::fixed_28v_5a);
    caps.push_back(pdos::fixed_36v_5a);

    caps.push_back(pdos::epr_avs_15_36v_240w);
    return caps;
}

} // namespace

CTS_TEST("TEST.PD.EPR.SNK3", "12_Negative_Testing_on_EPR_Request", REV3) {
    // Step 2 is moved after the EPR contract in step 12. UUT sends EPR_Mode
    // (Enter) right after the SPR contract, and any other message during EPR
    // Mode entry causes Soft Reset ([rev3.2 v1.2] 9.2.26.3.2). In the first phase
    // there is no EPR contract, the EPR_Request is rejected.

    // Steps 1-8: prepare the first EPR_Request, before an EPR contract exists.
    auto epr_request = start_epr_negotiation(t);
    ASSERT_TRUE(epr_request);

    // Step 9: Reject without an EPR contract
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Reject));

    // Step 10
    ASSERT_TRUE(t.expect_hard_reset(tProtErrHardReset, epr_request->time)); // [#6]

    // Step 11: reconnect and repeat steps 1-8.
    t.set_vbus(false);
    t.set_rp(TCPC_CC_LEVEL::NONE);
    t.wait(tCtsAfterLastPoint.max);
    epr_request = start_epr_negotiation(t);
    ASSERT_TRUE(epr_request);

    // Step 12: establish an EPR contract before rejecting the next Request.
    ASSERT_TRUE(accept_epr_request(t));

    // Step 2 (moved) [#2]
    ASSERT_TRUE(t.send_ext_ctrl_msg(PD_EXT_CTRL_MSGT::EPR_Get_Sink_Cap));
    ASSERT_TRUE(t.expect_ext_msg(PD_EXT_MSGT::EPR_Sink_Capabilities, tReceiverResponse));

    ASSERT_TRUE(t.send_epr_source_caps(epr_source_caps_changed()));

    // Step 13
    epr_request = t.expect_data_msg(PD_DATA_MSGT::EPR_Request, tReceiverResponse); // [#7]
    ASSERT_TRUE(epr_request);

    // Step 14: Reject with an EPR contract
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Reject));

    // Step 15. CTS checks only the absence of a Hard Reset, but with an EPR
    // contract in place Reject must keep the UUT in Ready ([rev3.2 v1.2] 9.2.4.5),
    // so nothing at all is expected here.
    EXPECT_FALSE(t.receive(tProtErrHardReset.max, epr_request->time));
}
