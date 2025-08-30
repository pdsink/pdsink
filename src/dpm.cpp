#include "common_macros.h"
#include "dpm.h"
#include "pd_log.h"
#include "port.h"
#include "utils/dobj_utils.h"

namespace pd {

using namespace dobj_utils;

//
// Custom handlers. You are expected to override these in your application.
//

auto DPM::get_sink_pdo_list() -> PDO_LIST {
    // SNK demands are filled only once and MUST NOT be changed
    if (sink_pdo_list.size() > 0) { return sink_pdo_list; }

    //
    // By default demand as much as possible. In other case SRC can
    // hide some capabilities. This list does NOT depend on SRC and
    // exists to describe SNK needs.
    //
    // NOTE: this can be just a list of uint32_t constants, but for better
    // readability we use PDO structures to fill data.
    //

    //
    // SPR PDOs first. Fixed ones first, ordered by voltage. Then PPS.
    //

    // See [rev3.2] 6.4.1.3 Sink Power Data Objects
    SNK_PDO_FIXED pdo1{create_pdo_variant_bits(PDO_VARIANT::FIXED)};

    // PDO 1 is always vSafe5V, with extra flags to describe demands.
    // NOTE: These flags should be zero in following PDO-s
    pdo1.dual_role_power = 0; // No DRP support, Sink only
    pdo1.higher_capability = 1; // Require more power for full functionality
    pdo1.unconstrained_power = 1;
    if (has_usb_comm()) { pdo1.usb_comms_capable = 1; }
    pdo1.dual_role_data = 0; // No DFP support, UFP only
    pdo1.frs_required = 0; // No FRS support

    set_snk_pdo_limits(pdo1.raw_value, PDO_LIMITS().set_mv(5000).set_ma(3000));
    sink_pdo_list.push_back(pdo1.raw_value);

    //
    // Fill the rest SRP PDOs, 2...7
    //
    SNK_PDO_FIXED pdo2{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo2.raw_value, PDO_LIMITS().set_mv(9000).set_ma(3000));
    sink_pdo_list.push_back(pdo2.raw_value);

    SNK_PDO_FIXED pdo3{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo3.raw_value, PDO_LIMITS().set_mv(12000).set_ma(3000));
    sink_pdo_list.push_back(pdo3.raw_value);

    SNK_PDO_FIXED pdo4{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo4.raw_value, PDO_LIMITS().set_mv(15000).set_ma(3000));
    sink_pdo_list.push_back(pdo4.raw_value);

    SNK_PDO_FIXED pdo5{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo5.raw_value, PDO_LIMITS().set_mv(20000).set_ma(5000));
    sink_pdo_list.push_back(pdo5.raw_value);

    // Before rev3.2 min PPS voltage was 3.3v, then updated to 5v.
    SNK_PDO_SPR_PPS pdo6{create_pdo_variant_bits(PDO_VARIANT::APDO_PPS)};
    set_snk_pdo_limits(pdo6.raw_value,
        PDO_LIMITS().set_mv_min(5000).set_mv_max(11000).set_ma(3000));
    sink_pdo_list.push_back(pdo6.raw_value);

    SNK_PDO_SPR_PPS pdo7{create_pdo_variant_bits(PDO_VARIANT::APDO_PPS)};
    set_snk_pdo_limits(pdo7.raw_value,
        PDO_LIMITS().set_mv_min(5000).set_mv_max(21000).set_ma(5000));
    sink_pdo_list.push_back(pdo7.raw_value);

    //
    // EPR PDOs. MUST start from 8. If SPR PDOs count < 7, the gap MUST be
    // padded with zeros. EPR block can have up to 3 Fixed PDOs + 1 AVS.
    //

    SNK_PDO_FIXED pdo8{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo8.raw_value, PDO_LIMITS().set_mv(28000).set_ma(5000));
    sink_pdo_list.push_back(pdo8.raw_value);

    SNK_PDO_FIXED pdo9{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo9.raw_value, PDO_LIMITS().set_mv(36000).set_ma(5000));
    sink_pdo_list.push_back(pdo9.raw_value);

    SNK_PDO_FIXED pdo10{create_pdo_variant_bits(PDO_VARIANT::FIXED)};
    set_snk_pdo_limits(pdo10.raw_value, PDO_LIMITS().set_mv(48000).set_ma(5000));
    sink_pdo_list.push_back(pdo10.raw_value);

    SNK_PDO_EPR_AVS pdo11{create_pdo_variant_bits(PDO_VARIANT::APDO_EPR_AVS)};
    set_snk_pdo_limits(pdo11.raw_value,
        PDO_LIMITS().set_mv_min(15000).set_mv_max(50000).set_pdp(get_epr_watts()));
    sink_pdo_list.push_back(pdo11.raw_value);

    return sink_pdo_list;
}

void DPM::fill_rdo_flags(uint32_t &rdo) {
    // Fill common RDO flags here
    // This is default implementation. You can override it if required.

    RDO_ANY rdo_bits{rdo};

    rdo_bits.epr_capable = 1;
    // Unchunked extended messages (long transfers) are NOT supported (and not
    // needed, because chunking is enough).
    // DON'T try to set this bit, it will fuckup everything!
    rdo_bits.unchunked_ext_msg_supported = 0;
    rdo_bits.no_usb_suspend = 1;
    rdo_bits.usb_comm_capable = has_usb_comm() ? 1 : 0;

    rdo = rdo_bits.raw_value;
}

// This one is called when `SRC Capabilities` and `EPR SRC Capabilities`
// are received from power source. Returns RDO and appropriate PDO as pair
auto DPM::get_request_data_object(const etl::ivector<uint32_t>& src_caps) -> etl::pair<uint32_t, uint32_t> {
    // This is default stub implementation, with simple trigger support.
    // Customize if required.

    //
    // NOTE: Here we indirectly check EPR mode. At the start we go to SRP, where
    // EPR voltage will not be available, and fallback to vSafe5V will be used.
    // Then, PE will automatically upgrade to EPR (with new EPR Caps) and this
    // function will be called again.
    //

    if (src_caps.empty()) {
        DPM_LOGE("get_request_data_object: invalid SRC Caps input");
        return {0, 0};
    }

    for (int i = 0, max = src_caps.size(); i < max; i++) {
        const auto pdo = src_caps[i];
        // Skip padded positions
        if (pdo == 0) { continue; }

        auto id = get_src_pdo_variant(pdo);
        if (id == PDO_VARIANT::UNKNOWN) { continue; }
        if (!trigger_any_pdo && id != trigger_pdo_variant) { continue; }

        if (!match_limits(pdo, trigger_mv, trigger_ma)) { continue; }

        // Create RDO
        RDO_ANY rdo{};
        fill_rdo_flags(rdo.raw_value);
        rdo.obj_position = i + 1; // zero-based index + 1

        // fill PDO-specific fields (volts/current/watts)
        auto limits = get_src_pdo_limits(pdo);

        if (id == PDO_VARIANT::FIXED) {
            uint32_t ma = trigger_ma ? trigger_ma : limits.ma;
            set_rdo_limits_fixed(rdo.raw_value, ma, ma);
            return {rdo.raw_value, pdo};
        }

        if (id == PDO_VARIANT::APDO_PPS) {
            uint32_t ma = trigger_ma ? trigger_ma : limits.ma;
            set_rdo_limits_pps(rdo.raw_value, trigger_mv, ma);
            return {rdo.raw_value, pdo};
        }

        if (id == PDO_VARIANT::APDO_SPR_AVS) {
            uint32_t ma = trigger_ma ? trigger_ma : limits.ma;
            set_rdo_limits_avs(rdo.raw_value, trigger_mv, ma);
            return {rdo.raw_value, pdo};
        }

        if (id == PDO_VARIANT::APDO_EPR_AVS) {
            uint32_t ma = trigger_ma;
            if (ma == 0) {
                ma = limits.pdp * 1000U / trigger_mv;
                if (ma > 5000) { ma = 5000; }
            }
            set_rdo_limits_avs(rdo.raw_value, trigger_mv, ma);
            return {rdo.raw_value, pdo};
        }
    }

    // By default return vSafe5V, based on first entry in SRC capabilities
    const auto pdo = src_caps[0];
    RDO_FIXED rdo{};
    fill_rdo_flags(rdo.raw_value);
    rdo.obj_position = 0 + 1; // Position = zero-based index + 1

    auto limits = get_src_pdo_limits(pdo);
    set_rdo_limits_fixed(rdo.raw_value, limits.ma, limits.ma);
    return {rdo.raw_value, pdo};
}

void DPM::request_new_power_level() {
    // Only if explicit contract exists.
    // If not - data will be used at handshake.
    if (port.pe_flags.test(PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
        port.dpm_requests.set(DPM_REQUEST_FLAG::NEW_POWER_LEVEL);
        // Don't call wakeup() to keep execution context in driver's "thread".
        // Rely on timer's periodic tick to catch the request.
    }
}

void DPM::trigger(PDO_VARIANT pdo_variant, uint32_t mv, uint32_t ma) {
    trigger_mv = mv;
    trigger_ma = ma;
    trigger_pdo_variant = pdo_variant;
    trigger_any_pdo = (pdo_variant == PDO_VARIANT::UNKNOWN);
}

} // namespace pd
