#include "common_macros.h"
#include "dpm.h"
#include "pe.h"
#include "sink.h"

namespace pd {

DPM::DPM(Sink& sink, PE& pe) : sink{sink}, pe{pe} {
    sink.dpm = this;
};

//
// Custom handlers. You are expected to override these in your application.
//

auto DPM::get_sink_pdo_list() -> PDO_LIST {
    // SNK demands are filled only once and MUST NOT be changed
    if (sink_rdo_list.size() > 0) { return sink_rdo_list; }

    //
    // By default demand as much as possible. In other case SRC can
    // hide some capabilities. Note, this list does NOT depends on SRC and
    // exists to describe SNK needs.
    //
    // Note, this can be just a list of uint32_t constants, but for better
    // readability we use PDO structures to fill data.
    //

    //
    // SPR PDOs first. Fixed ones first, ordered by voltage. Then PPS.
    //

    // See [rev3.2] 6.4.1.3 Sink Power Data Objects
    // PDO 1 is always vSafe5V, with extra flags to describe demands.
    // Note, those flags should be zero in following PDO-s
    SNK_PDO_FIXED pdo1{};
    pdo1.pdo_type = PDO_TYPE::FIXED;
    pdo1.dual_role_power = 0; // No DRP support, Sink only
    pdo1.higher_capability = 1; // Require more power for full functionality
    pdo1.unconstrained_power = 1;
    if (has_usb_comm()) { pdo1.usb_comms_capable = 1; }
    pdo1.dual_role_data = 0; // No DFP support, UFP only
    pdo1.frs_required = 0; // No FRS support

    pdo1.voltage = 5000 / 50; // 5V
    pdo1.max_current = 3000 / 10; // 3A

    sink_rdo_list.push_back(pdo1.raw_value);

    //
    // Fill the rest SRP PDOs, 2...7
    //
    SNK_PDO_FIXED pdo2{};
    pdo2.pdo_type = PDO_TYPE::FIXED;
    pdo2.voltage = 9000 / 50; // 9V
    pdo2.max_current = 3000 / 10; // 3A

    sink_rdo_list.push_back(pdo2.raw_value);

    SNK_PDO_FIXED pdo3{};
    pdo3.pdo_type = PDO_TYPE::FIXED;
    pdo3.voltage = 12000 / 50; // 12V
    pdo3.max_current = 3000 / 10; // 3A

    sink_rdo_list.push_back(pdo3.raw_value);

    SNK_PDO_FIXED pdo4{};
    pdo4.pdo_type = PDO_TYPE::FIXED;
    pdo4.voltage = 15000 / 50; // 15V
    pdo4.max_current = 3000 / 10; // 3A

    sink_rdo_list.push_back(pdo4.raw_value);

    SNK_PDO_FIXED pdo5{};
    pdo5.pdo_type = PDO_TYPE::FIXED;
    pdo5.voltage = 20000 / 50; // 20V
    pdo5.max_current = 5000 / 10; // 5A

    sink_rdo_list.push_back(pdo5.raw_value);

    SNK_PDO_SPR_PPS pdo6{};
    pdo6.pdo_type = PDO_TYPE::AUGMENTED;
    pdo6.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_PPS;
    pdo6.min_voltage = 3300 / 100; // 3.3V
    pdo6.max_voltage = 11000 / 100; // 11V
    pdo6.max_current = 3000 / 50; // 3A

    sink_rdo_list.push_back(pdo6.raw_value);

    SNK_PDO_SPR_PPS pdo7{};
    pdo7.pdo_type = PDO_TYPE::AUGMENTED;
    pdo7.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_PPS;
    pdo7.min_voltage = 3300 / 100; // 3.3V
    pdo7.max_voltage = 21000 / 100; // 21V
    pdo7.max_current = 5000 / 50; // 5A

    sink_rdo_list.push_back(pdo7.raw_value);

    //
    // EPR PDOs. MUST start from 8. If SPR PDOs count < 7, the gap must be
    // padded with zeros. EPR block an have up to 3 Fixed PDOs + 1 AVS.
    //

    SNK_PDO_FIXED pdo8{};
    pdo8.pdo_type = PDO_TYPE::FIXED;
    pdo8.voltage = 28000 / 50; // 28V
    pdo8.max_current = 5000 / 10; // 5A

    sink_rdo_list.push_back(pdo8.raw_value);

    SNK_PDO_FIXED pdo9{};
    pdo9.pdo_type = PDO_TYPE::FIXED;
    pdo9.voltage = 36000 / 50; // 36V
    pdo9.max_current = 5000 / 10; // 5A

    sink_rdo_list.push_back(pdo9.raw_value);

    SNK_PDO_FIXED pdo10{};
    pdo10.pdo_type = PDO_TYPE::FIXED;
    pdo10.voltage = 48000 / 50; // 48V
    pdo10.max_current = 5000 / 10; // 5A

    sink_rdo_list.push_back(pdo10.raw_value);

    SNK_PDO_EPR_AVS pdo11{};
    pdo11.pdo_type = PDO_TYPE::AUGMENTED;
    pdo11.apdo_subtype = PDO_AUGMENTED_SUBTYPE::EPR_AVS;
    pdo11.min_voltage = 15000 / 100; // 15V
    pdo11.max_voltage = 50000 / 100; // 50V
    pdo11.pdp = 240 / 1; // 240W

    sink_rdo_list.push_back(pdo11.raw_value);

    return sink_rdo_list;
}

void DPM::fill_rdo_flags(uint32_t &rdo) {
    RDO_ANY rdo_bits{rdo};

    // Fill RDO flags here
    rdo_bits.epr_capable = 1;
    // Unchunked ext messages (long transfers) NOT supported (and not needed,
    // because chunking is enough).
    // Don't try to set this bit, it will fuckup everything!
    rdo_bits.unchunked_ext_msg_supported = 0;
    rdo_bits.no_usb_suspend = 1;
    rdo_bits.usb_comm_capable = has_usb_comm() ? 1 : 0;

    rdo = rdo_bits.raw_value;
}

// This one is called when `SRC Capabilities` and `EPR SRC Capabilities`
// are received from power source. Returns RDO and appropriate PDO as pair
auto DPM::get_request_data_object(const etl::ivector<uint32_t>& src_caps) -> uint32_t {
    // This is default stub implementation, with simple trigger support.
    // Customize if required.

    //
    // Note, here we indirectly check EPR mode. At the start we go to SRP, where
    // EPR voltage will not be available, and fallback to vSafe5V will be used.
    // Then, PE will automatically upgrade to EPR (with new EPR Caps) and this
    // function will be called again.
    //

    if (trigger_mode == TRIGGER_MODE::FIXED) {
        for (int i = 0, max = src_caps.size(); i < max; i++) {
            const auto pdo = src_caps[i];
            // Skip padded positions
            if (pdo == 0) { continue; }
            // Skip not fixed PDOs
            if (!DO_TOOLS::is_fixed(pdo)) { continue; }

            const PDO_FIXED pdo_bits{pdo};
            const uint32_t pdo_mv = static_cast<uint32_t>(pdo_bits.voltage) * 50U;
            const uint32_t pdo_ma = static_cast<uint32_t>(pdo_bits.max_current) * 10U;

            if (pdo_mv == trigger_mv) {
                RDO_FIXED rdo{};
                fill_rdo_flags(rdo.raw_value);
                rdo.obj_position = i + 1; // zero-based index + 1
                rdo.max_current = pdo_ma / 10U;
                rdo.operating_current = pdo_ma / 10U;

                return rdo.raw_value;
            }
        }
    }

    if (trigger_mode == TRIGGER_MODE::SPR_PPS) {
        for (int i = 0, max = src_caps.size(); i < max; i++) {
            const auto pdo = src_caps[i];
            // Skip padded positions
            if (pdo == 0) { continue; }
            // Skip not SPR_PPS PDOs
            if (!DO_TOOLS::is_spr_pps(pdo)) { continue; }

            const PDO_SPR_PPS pdo_bits{pdo};
            const uint32_t pdo_max_mv = static_cast<uint32_t>(pdo_bits.max_voltage) * 100U;
            const uint32_t pdo_max_ma = static_cast<uint32_t>(pdo_bits.max_current) * 50U;

            if (pdo_max_mv >= trigger_mv && pdo_max_ma >= trigger_ma) {
                RDO_PPS rdo{};
                fill_rdo_flags(rdo.raw_value);
                rdo.obj_position = i + 1; // zero-based index + 1
                rdo.operating_current = trigger_ma / 50U;
                rdo.output_voltage = trigger_mv / 20U;

                return rdo.raw_value;
            }
        }
    }

    // By default return vSafe5V, based on first entry in SRC capabilities
    RDO_FIXED rdo{};
    fill_rdo_flags(rdo.raw_value);
    rdo.obj_position = 0 + 1; // Position = zero-based index + 1

    if (DO_TOOLS::is_fixed(src_caps[0])) {
        const PDO_FIXED pdo_bits{src_caps[0]};
        const uint32_t pdo_ma = static_cast<uint32_t>(pdo_bits.max_current) * 10U;
        rdo.max_current = pdo_ma / 10U;
        rdo.operating_current = pdo_ma / 10U;
    };

    return rdo.raw_value;
}

void DPM::trigger_fixed(uint32_t mv) {
    trigger_mv = mv;
    trigger_mode = FIXED;

    // If explicit contract already exists, request new power level
    if (pe.flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
        pe.dpm_requests.set(DPM_REQUEST::NEW_POWER_LEVEL);
    }
}
void DPM::trigger_spr_pps(uint32_t mv, uint32_t ma) {
    trigger_mv = mv;
    trigger_ma = ma;
    trigger_mode = SPR_PPS;

    // If explicit contract already exists, request new power level
    if (pe.flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
        pe.dpm_requests.set(DPM_REQUEST::NEW_POWER_LEVEL);
    }
}


} // namespace pd
