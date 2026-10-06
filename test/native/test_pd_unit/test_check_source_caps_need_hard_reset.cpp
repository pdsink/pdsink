#include <gtest/gtest.h>
#include "pd/pe.h"
#include "pd/data_objects.h"

using namespace pd;

uint32_t make_fixed_pdo(uint32_t voltage_mv, uint32_t current_ma) {
    PDO_FIXED pdo{};
    pdo.pdo_type = PDO_TYPE::FIXED;
    pdo.voltage = voltage_mv / 50;
    pdo.max_current = current_ma / 10;
    return pdo.raw_value;
}

uint32_t make_epr_avs_apdo(uint32_t min_voltage_mv, uint32_t max_voltage_mv, uint32_t pdp_watts) {
    PDO_EPR_AVS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::EPR_AVS;
    pdo.min_voltage = min_voltage_mv / 100;
    pdo.max_voltage = max_voltage_mv / 100;
    pdo.pdp = pdp_watts;
    return pdo.raw_value;
}

TEST(SourceCapsHardResetTest, EmptyListRequiresHardReset) {
    PDO_LIST caps;
    EXPECT_TRUE(PE::check_source_caps_need_hard_reset(caps, false));
    EXPECT_TRUE(PE::check_source_caps_need_hard_reset(caps, true));
}

TEST(SourceCapsHardResetTest, EprPdoRequiresHardResetOnlyAtSprPositionInEprMode) {
    for (auto pdo : {make_fixed_pdo(28000, 5000), make_epr_avs_apdo(15000, 28000, 140)}) {
        PDO_LIST caps{make_fixed_pdo(5000, 3000)};
        caps.resize(MaxPdoObjects_SPR);
        caps.back() = pdo;
        EXPECT_FALSE(PE::check_source_caps_need_hard_reset(caps, false));
        EXPECT_TRUE(PE::check_source_caps_need_hard_reset(caps, true));

        caps.back() = 0;
        caps.push_back(pdo);
        EXPECT_FALSE(PE::check_source_caps_need_hard_reset(caps, true));
    }
}

TEST(SourceCapsHardResetTest, OtherErrorsDoNotRequireHardReset) {
    PDO_LIST caps{
        make_fixed_pdo(9000, 3000),
        make_fixed_pdo(5000, 3000),
        make_fixed_pdo(5000, 3000)
    };
    EXPECT_FALSE(PE::check_source_caps_need_hard_reset(caps, false));
    EXPECT_FALSE(PE::check_source_caps_need_hard_reset(caps, true));
}
