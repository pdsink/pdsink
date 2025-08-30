#pragma once

#include "data_objects.h"

namespace pd {
namespace dobj_utils {

enum class PDO_VARIANT {
    UNKNOWN,
    FIXED,
    APDO_PPS,
    APDO_SPR_AVS,
    APDO_EPR_AVS
};

class PDO_LIMITS {
public:
    uint32_t mv_min{0};
    uint32_t mv_max{0};
    uint32_t ma{0};
    uint32_t pdp{0}; // in watts, for EPR_AVS, optional

    PDO_LIMITS& set_mv_min(uint32_t mv) { mv_min = mv; return *this; }
    PDO_LIMITS& set_mv_max(uint32_t mv) { mv_max = mv; return *this; }
    PDO_LIMITS& set_ma(uint32_t ma) { this->ma = ma; return *this; }
    PDO_LIMITS& set_pdp(uint32_t pdp) { this->pdp = pdp; return *this; }

    // Sugar for fixed objects
    PDO_LIMITS& set_mv(uint32_t mv) { mv_min = mv; mv_max = mv; return *this; }
};

PDO_VARIANT get_src_pdo_variant(uint32_t src_pdo);

inline PDO_VARIANT get_snk_pdo_variant(uint32_t snk_pdo) {
    // WARNING: in spec rev3.2 v1.1, SNK BATTERY/VARIABLE IDs seem swapped
    // Be careful if decide to add support.
    return get_src_pdo_variant(snk_pdo);
};

PDO_LIMITS get_src_pdo_limits(uint32_t src_pdo);

void set_snk_pdo_limits(uint32_t& snk_pdo, const PDO_LIMITS& limits);

bool match_limits(uint32_t pdo, uint32_t mv, uint32_t ma);

uint32_t create_pdo_variant_bits(PDO_VARIANT id);

inline void set_rdo_limits_fixed(uint32_t& rdo, uint32_t operating_ma, uint32_t max_ma) {
    RDO_FIXED rdo_bits{rdo};
    rdo_bits.max_current = max_ma / 10u;
    rdo_bits.operating_current = operating_ma / 10u;
    rdo = rdo_bits.raw_value;
}

inline void set_rdo_limits_pps(uint32_t& rdo, uint32_t mv, uint32_t ma) {
    RDO_PPS rdo_bits{rdo};
    rdo_bits.output_voltage = mv / 20u;
    rdo_bits.operating_current = ma / 50u;
    rdo = rdo_bits.raw_value;
}

inline void set_rdo_limits_avs(uint32_t& rdo, uint32_t mv, uint32_t ma) {
    RDO_AVS rdo_bits{rdo};
    // The spec says the step is 25 mV, but the two least significant bits must be zero
    rdo_bits.output_voltage = (mv / 100u) << 2;
    rdo_bits.operating_current = ma / 50u;
    rdo = rdo_bits.raw_value;
}

} // namespace dobj_utils
} // namespace pd
