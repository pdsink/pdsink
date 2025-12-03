#include <gtest/gtest.h>
#include "pd/pe.h"
#include "pd/data_objects.h"

using namespace pd;

// Helper functions to create PDOs for testing

uint32_t make_fixed_pdo(uint32_t voltage_mv, uint32_t current_ma) {
    PDO_FIXED pdo{};
    pdo.pdo_type = PDO_TYPE::FIXED;
    pdo.voltage = voltage_mv / 50;  // Convert mV to 50mV units
    pdo.max_current = current_ma / 10;  // Convert mA to 10mA units
    return pdo.raw_value;
}

uint32_t make_pps_apdo(uint32_t min_voltage_mv, uint32_t max_voltage_mv, uint32_t current_ma) {
    PDO_SPR_PPS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_PPS;
    pdo.min_voltage = min_voltage_mv / 100;  // Convert mV to 100mV units
    pdo.max_voltage = max_voltage_mv / 100;  // Convert mV to 100mV units
    pdo.max_current = current_ma / 50;  // Convert mA to 50mA units
    return pdo.raw_value;
}

uint32_t make_spr_avs_apdo(uint32_t current_15v_ma, uint32_t current_20v_ma) {
    PDO_SPR_AVS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_AVS;
    pdo.max_current_15v = current_15v_ma / 10;  // Convert mA to 10mA units
    pdo.max_current_20v = current_20v_ma / 10;  // Convert mA to 10mA units
    return pdo.raw_value;
}

uint32_t make_epr_avs_apdo(uint32_t min_voltage_mv, uint32_t max_voltage_mv, uint32_t pdp_watts) {
    PDO_EPR_AVS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::EPR_AVS;
    pdo.min_voltage = min_voltage_mv / 100;  // Convert mV to 100mV units
    pdo.max_voltage = max_voltage_mv / 100;  // Convert mV to 100mV units
    pdo.pdp = pdp_watts;  // Watts
    return pdo.raw_value;
}

// Group 1: Basic validation

TEST(ValidateSourceCapsTest, EmptyListFails) {
    PDO_LIST caps;
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, FirstPdoMustBeVSafe5V) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(9000, 3000)); // 9V, 3A - wrong
    EXPECT_FALSE(PE::validate_source_caps(caps));

    caps.clear();
    caps.push_back(make_fixed_pdo(5000, 3000)); // 5V, 3A - correct
    EXPECT_TRUE(PE::validate_source_caps(caps));
}

// Group 2: Count limitations

TEST(ValidateSourceCapsTest, SprMaximum7Pdos) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));   // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));   // 9V, 3A
    caps.push_back(make_fixed_pdo(12000, 3000));  // 12V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000));  // 15V, 3A
    caps.push_back(make_fixed_pdo(20000, 3000));  // 20V, 3A
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A
    EXPECT_TRUE(PE::validate_source_caps(caps)); // 7 PDO - OK

    caps.push_back(make_pps_apdo(5000, 16000, 3000)); // 8th PDO without EPR
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, EprMaximum11Pdos) {
    PDO_LIST caps;
    // 7 SPR PDO
    caps.push_back(make_fixed_pdo(5000, 3000));   // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));   // 9V, 3A
    caps.push_back(make_fixed_pdo(12000, 3000));  // 12V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000));  // 15V, 3A
    caps.push_back(make_fixed_pdo(20000, 3000));  // 20V, 3A
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A

    // 4 EPR PDO (positions 8-11)
    caps.push_back(make_fixed_pdo(28000, 5000));  // 28V, 5A (EPR)
    caps.push_back(make_fixed_pdo(36000, 5000));  // 36V, 5A (EPR)
    caps.push_back(make_fixed_pdo(48000, 5000));  // 48V, 5A (EPR)
    caps.push_back(make_epr_avs_apdo(15000, 48000, 240)); // EPR AVS 15-48V, 240W
    EXPECT_TRUE(PE::validate_source_caps(caps)); // 11 PDO - OK

    caps.push_back(make_fixed_pdo(5000, 1000)); // 12th PDO
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

// Group 3: AVS limitations

TEST(ValidateSourceCapsTest, OnlyOneSprAvsAllowed) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_spr_avs_apdo(3000, 3000)); // SPR AVS 3A@15V, 3A@20V
    EXPECT_TRUE(PE::validate_source_caps(caps));

    caps.push_back(make_spr_avs_apdo(2500, 2500)); // SPR AVS #2
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, OnlyOneEprAvsAllowed) {
    PDO_LIST caps;
    // SPR PDO (positions 1-7)
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));  // 9V, 3A
    caps.push_back(make_fixed_pdo(12000, 3000)); // 12V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000)); // 15V, 3A
    caps.push_back(make_fixed_pdo(20000, 3000)); // 20V, 3A
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A

    // EPR PDO (positions 8+)
    caps.push_back(make_fixed_pdo(28000, 5000));  // 28V, 5A (EPR, position 8)
    caps.push_back(make_epr_avs_apdo(15000, 28000, 140)); // EPR AVS 15-28V, 140W
    EXPECT_TRUE(PE::validate_source_caps(caps));

    caps.push_back(make_epr_avs_apdo(15000, 36000, 180)); // EPR AVS #2
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

// Group 4: Fixed PDO duplicate check

TEST(ValidateSourceCapsTest, NoDuplicateFixedVoltages) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 2000));  // 9V, 2A
    EXPECT_TRUE(PE::validate_source_caps(caps));

    caps.push_back(make_fixed_pdo(9000, 3000));  // 9V, 3A - duplicate!
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, DuplicatePpsAllowed) {
    // Spec doesn't prohibit duplicates for APDO
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 1500)); // PPS 5-21V, 1.5A (different current)
    EXPECT_TRUE(PE::validate_source_caps(caps));
}

// Group 5: Fixed PDO sorting

TEST(ValidateSourceCapsTest, FixedPdoMustBeSortedByVoltage) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));  // 9V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000)); // 15V, 3A
    EXPECT_TRUE(PE::validate_source_caps(caps)); // Correct sorting

    caps.clear();
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000)); // 15V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));  // 9V, 3A - wrong order!
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, FirstFixedAlways5V) {
    // First must always be 5V, rest in ascending order
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_fixed_pdo(5000, 5000));  // 5V, 5A - duplicate voltage not allowed
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

// Group 6: PPS APDO sorting

TEST(ValidateSourceCapsTest, PpsMustBeSortedByMaxVoltage) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 16000, 3000)); // PPS 5-16V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A
    EXPECT_TRUE(PE::validate_source_caps(caps)); // Correct sorting

    caps.clear();
    caps.push_back(make_fixed_pdo(5000, 3000));  // 5V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A - wrong order!
    EXPECT_FALSE(PE::validate_source_caps(caps));
}

// Group 7: Complex tests

TEST(ValidateSourceCapsTest, ValidSprCapabilities) {
    PDO_LIST caps;
    caps.push_back(make_fixed_pdo(5000, 3000));   // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));   // 9V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000));  // 15V, 3A
    caps.push_back(make_fixed_pdo(20000, 3000));  // 20V, 3A
    caps.push_back(make_spr_avs_apdo(3000, 3000)); // SPR AVS 3A@15V, 3A@20V
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A
    EXPECT_TRUE(PE::validate_source_caps(caps));
}

TEST(ValidateSourceCapsTest, ValidEprCapabilities) {
    PDO_LIST caps;
    // SPR part (positions 1-7)
    caps.push_back(make_fixed_pdo(5000, 3000));    // 5V, 3A
    caps.push_back(make_fixed_pdo(9000, 3000));    // 9V, 3A
    caps.push_back(make_fixed_pdo(15000, 3000));   // 15V, 3A
    caps.push_back(make_fixed_pdo(20000, 3000));   // 20V, 3A
    caps.push_back(make_spr_avs_apdo(3000, 3000)); // SPR AVS 3A@15V, 3A@20V
    caps.push_back(make_pps_apdo(5000, 11000, 3000)); // PPS 5-11V, 3A
    caps.push_back(make_pps_apdo(5000, 21000, 3000)); // PPS 5-21V, 3A

    // EPR part (positions 8-11)
    caps.push_back(make_fixed_pdo(28000, 5000));   // 28V, 5A (EPR)
    caps.push_back(make_fixed_pdo(36000, 5000));   // 36V, 5A (EPR)
    caps.push_back(make_fixed_pdo(48000, 5000));   // 48V, 5A (EPR)
    caps.push_back(make_epr_avs_apdo(15000, 48000, 240)); // EPR AVS 15-48V, 240W
    EXPECT_TRUE(PE::validate_source_caps(caps));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
