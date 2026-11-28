#include "harness/tester.h"

using namespace cts;
using namespace pd;

namespace {

// Step 5: 240W source for PD3, 100W for PD2
auto source_caps(PD_REVISION::Type revision) -> std::vector<uint32_t> {
    const uint32_t first_pdo = pdos::fixed_5v_3a | pdos::dual_role_power |
        (revision != PD_REVISION::REV20 ? pdos::epr_capable : 0u);
    std::vector<uint32_t> caps{
        first_pdo, pdos::fixed_9v_3a, pdos::fixed_15v_3a, pdos::fixed_20v_5a
    };

    if (revision != PD_REVISION::REV20) {
        caps.push_back(pdos::spr_avs_15v_3a_20v_5a);
        caps.push_back(pdos::pps_5_21v_5a);
    }
    return caps;
}

// SKEDB layout, [rev3.2 v1.2] Table 6.61
auto skedb_bytes(const SKEDB& skedb) -> std::vector<uint8_t> {
    return {
        uint8_t(skedb.vid), uint8_t(skedb.vid >> 8),
        uint8_t(skedb.pid), uint8_t(skedb.pid >> 8),
        uint8_t(skedb.xid), uint8_t(skedb.xid >> 8), uint8_t(skedb.xid >> 16), uint8_t(skedb.xid >> 24),
        skedb.fw_version, skedb.hw_version, skedb.skedb_version, skedb.load_step.raw_value,
        uint8_t(skedb.sink_load_characteristics.raw_value),
        uint8_t(skedb.sink_load_characteristics.raw_value >> 8),
        skedb.compliance.raw_value, skedb.touch_temp, skedb.battery_info.raw_value,
        skedb.sink_modes.raw_value,
        skedb.spr_sink_minimum_pdp, skedb.spr_sink_operational_pdp, skedb.spr_sink_maximum_pdp,
        skedb.epr_sink_minimum_pdp, skedb.epr_sink_operational_pdp, skedb.epr_sink_maximum_pdp
    };
}

// Step 7: requested power is within SPR Sink Maximum PDP
void check_request_pdp(const Frame& request, const std::vector<uint32_t>& caps, const SKEDB& vif) {
    const auto rdo = request.chunk.read32(0);
    const auto position = RDO_ANY{rdo}.obj_position;
    ASSERT_TRUE(position >= 1 && position <= caps.size());

    const auto pdo = caps[position - 1];
    const uint32_t max_mw = vif.spr_sink_maximum_pdp * 1000u;
    uint32_t mv = 0;
    uint32_t ma = 0;

    if (PDO_FIXED{pdo}.pdo_type == PDO_TYPE::FIXED) { // [#12a]
        mv = PDO_FIXED{pdo}.voltage * 50u;
        ma = RDO_FIXED{rdo}.operating_current * 10u;
    } else if (PDO_SPR_PPS{pdo}.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_PPS) {
        EXPECT_TRUE(vif.sink_modes.pps_charging_supported); // [#12c]
        mv = RDO_PPS{rdo}.output_voltage * 20u;
        ma = RDO_PPS{rdo}.operating_current * 50u;
    } else {
        EXPECT_TRUE(vif.sink_modes.avs_supported); // [#12d]
        mv = RDO_AVS{rdo}.output_voltage * 25u;
        ma = RDO_AVS{rdo}.operating_current * 50u;
    }
    const uint32_t requested_mw = mv * ma / 1000u;
    EXPECT_LE(requested_mw, max_mw); // [#12]
}

} // namespace

CTS_TEST("TEST.PD.PS.SNK", "1_PDO_Transitions_As_Sink", REV_ALL) {
    // UUT VIF: Sink Capabilities Extended fields
    const auto vif = t.dpm.get_sink_cap_extended();
    const bool pd3 = !t.is_pd2();

    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Steps 2-3
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap));
    ASSERT_TRUE(t.expect_data_msg(PD_DATA_MSGT::Sink_Capabilities, tReceiverResponse)); // [#1]

    // Step 4
    if (pd3) {
        ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Sink_Cap_Extended));
        auto ext_caps = t.expect_ext_msg(PD_EXT_MSGT::Sink_Capabilities_Extended, tReceiverResponse); // [#19]
        ASSERT_TRUE(ext_caps);

        // COMMON.CHECK.PD.15
        const auto& data = ext_caps->msg.get_data();
        ASSERT_EQ(data.size(), 24u);
        EXPECT_EQ(std::vector<uint8_t>(data.begin(), data.end()), skedb_bytes(vif));
        EXPECT_EQ(vif.skedb_version, 1u);
    }

    // Step 5
    const auto caps = source_caps(t.revision());
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, caps));

    // Step 6
    auto request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [#2]
    ASSERT_TRUE(request);

    // Step 7
    ASSERT_NO_FATAL_FAILURE(check_request_pdp(*request, caps, vif));

    // Step 8. Request exactly at tSinkRequest min is allowed.
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Wait));
    const auto wait_sent = t.now();
    EXPECT_FALSE(t.receive(tSinkRequest.min - 1 * us)); // [#17]

    // Step 9
    request = t.receive(tCtsInfinite.max, wait_sent);
    if (!request) {
        ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::Source_Capabilities, caps));
        request = t.expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse);
    }
    ASSERT_TRUE(request && request->is_data(PD_DATA_MSGT::Request)); // [#18]

    // Steps 10-13. VBUS current checks are not modeled.
    ASSERT_TRUE(t.accept_request());

    // Step 14
    auto frame = t.receive(5000 * ms);
    if (!frame) { return; }

    ASSERT_TRUE(frame->is_epr_mode(EPR_MODE_ACTION::ENTER));

    // Step 16: EPR_Supported_As_Snk = Yes for PD3
    ASSERT_TRUE(pd3); // [#15]

    // Step 15: Enter Failed, Source unable to enter EPR Mode
    EPRMDO failed{};
    failed.action = EPR_MODE_ACTION::ENTER_FAILED;
    failed.data = 4;
    ASSERT_TRUE(t.send_data_msg(PD_DATA_MSGT::EPR_Mode, { failed.raw_value }));
    EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Soft_Reset, tProtErrSoftReset)); // [#16]
}
