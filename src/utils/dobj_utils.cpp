#include "dobj_utils.h"

namespace pd {
namespace dobj_utils {

SRCSNK_PDO_ID get_src_pdo_id(uint32_t src_pdo) {
    PDO_EPR_AVS pdo{src_pdo}; // Use this type to just access high bits
    if (pdo.pdo_type == PDO_TYPE::FIXED) { return SRCSNK_PDO_ID::FIXED; }
    if (pdo.pdo_type == PDO_TYPE::AUGMENTED) {
        if (pdo.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_PPS) { return SRCSNK_PDO_ID::SPR_PPS; }
        if (pdo.apdo_subtype == PDO_AUGMENTED_SUBTYPE::SPR_AVS) { return SRCSNK_PDO_ID::SPR_AVS; }
        if (pdo.apdo_subtype == PDO_AUGMENTED_SUBTYPE::EPR_AVS) { return SRCSNK_PDO_ID::EPR_AVS; }
    }
    return SRCSNK_PDO_ID::UNKNOWN;
}

PDO_LIMITS get_src_pdo_limits(uint32_t src_pdo) {
    PDO_LIMITS limits{};
    auto pdo_type = get_src_pdo_id(src_pdo);

    if (pdo_type == SRCSNK_PDO_ID::FIXED) {
        const PDO_FIXED pdo{src_pdo};
        return limits
            .set_mv(pdo.voltage * 50u)
            .set_ma(pdo.max_current * 10u);
    }

    if (pdo_type == SRCSNK_PDO_ID::SPR_PPS) {
        const PDO_SPR_PPS pdo{src_pdo};
        return limits
            .set_mv_min(pdo.min_voltage * 100u)
            .set_mv_max(pdo.max_voltage * 100u)
            .set_ma(pdo.max_current * 50u);
    }

    if (pdo_type == SRCSNK_PDO_ID::SPR_AVS) {
        const PDO_SPR_AVS pdo{src_pdo};
        auto mv_max = 15000u; // 15V
        auto ma = pdo.max_current_15v * 10u;
        if (pdo.max_current_20v > 0) {
            mv_max = 20000u; // 20V
            ma = pdo.max_current_20v * 10u;
        }
        return limits
            .set_mv_min(9000u)
            .set_mv_max(mv_max)
            .set_ma(ma);
    }

    if (pdo_type == SRCSNK_PDO_ID::EPR_AVS) {
        const PDO_EPR_AVS pdo{src_pdo};
        return limits
            .set_mv_min(pdo.min_voltage * 100u)
            .set_mv_max(pdo.max_voltage * 100u)
            .set_pdp(pdo.pdp);
    }

    return limits;
}

void set_snk_pdo_limits(uint32_t& snk_pdo, const PDO_LIMITS& limits) {
    auto id = get_snk_pdo_id(snk_pdo);
    if (id == SRCSNK_PDO_ID::FIXED) {
        PDO_FIXED pdo{snk_pdo};
        pdo.voltage = limits.mv_min / 50u;
        pdo.max_current = limits.ma / 10u;
        snk_pdo = pdo.raw_value;
        return;
    }

    if (id == SRCSNK_PDO_ID::SPR_PPS) {
        PDO_SPR_PPS pdo{snk_pdo};
        pdo.min_voltage = limits.mv_min / 100u;
        pdo.max_voltage = limits.mv_max / 100u;
        pdo.max_current = limits.ma / 50u;
        snk_pdo = pdo.raw_value;
        return;
    }

    if (id == SRCSNK_PDO_ID::SPR_AVS) {
        PDO_SPR_AVS pdo{snk_pdo};
        pdo.max_current_15v = limits.ma / 10u;
        pdo.max_current_20v = limits.ma / 10u;
        snk_pdo = pdo.raw_value;
        return;
    }

    if (id == SRCSNK_PDO_ID::EPR_AVS) {
        PDO_EPR_AVS pdo{snk_pdo};
        pdo.min_voltage = limits.mv_min / 100u;
        pdo.max_voltage = limits.mv_max / 100u;
        pdo.pdp = limits.pdp;
        snk_pdo = pdo.raw_value;
        return;
    }
}

bool match_limits(uint32_t pdo, uint32_t mv, uint32_t ma) {
    auto id = get_src_pdo_id(pdo);
    if (id == SRCSNK_PDO_ID::UNKNOWN) { return false; }

    auto limits = get_src_pdo_limits(pdo);
    // Voltage check is the same for all PDO kinds
    if (mv < limits.mv_min || mv > limits.mv_max) { return false; }
    // If no current limit - not more checks
    if (ma == 0) { return true; }

    if (id == SRCSNK_PDO_ID::FIXED ||
        id == SRCSNK_PDO_ID::SPR_PPS ||
        id == SRCSNK_PDO_ID::SPR_AVS)
    {
        return ma <= limits.ma;
    }

    if (id == SRCSNK_PDO_ID::EPR_AVS) {
        // For EPR_AVS current is not specified, only PDP
        if (limits.pdp == 0) { return true; }
        // Calculate implied current from PDP and voltage
        auto implied_ma = (limits.pdp * 1000u) / mv;
        return implied_ma >= ma;
    }

    return false;
}

uint32_t create_pdo_type_bits(SRCSNK_PDO_ID id) {
    // WARNING: in spec rev3.2 v1.1, SNK BATTERY/VARIABLE IDs seem swapped
    // Be careful if decide to add support.
    PDO_SPR_PPS pdo{};

    switch (id) {
        case SRCSNK_PDO_ID::FIXED:
            pdo.pdo_type = PDO_TYPE::FIXED;
            return pdo.raw_value;
        case SRCSNK_PDO_ID::SPR_PPS:
            pdo.pdo_type = PDO_TYPE::AUGMENTED;
            pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_PPS;
            return pdo.raw_value;
        case SRCSNK_PDO_ID::SPR_AVS:
            pdo.pdo_type = PDO_TYPE::AUGMENTED;
            pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_AVS;
            return pdo.raw_value;
        case SRCSNK_PDO_ID::EPR_AVS:
            pdo.pdo_type = PDO_TYPE::AUGMENTED;
            pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::EPR_AVS;
            return pdo.raw_value;
        case SRCSNK_PDO_ID::UNKNOWN:
            break;
    }

    return 0;
}

} // namespace dobj_utils
} // namespace pd
