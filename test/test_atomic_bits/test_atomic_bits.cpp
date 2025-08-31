#include <gtest/gtest.h>
#include "pd/utils/atomic_bits.h"

TEST(AtomicBitsTest, ConstructorInitializesToZero) {
    AtomicBits<32> bits;

    for (size_t i = 0; i < 32; ++i) {
        EXPECT_FALSE(bits.test(i));
    }
}

TEST(AtomicBitsTest, SetBit) {
    AtomicBits<32> bits;

    bits.set(5);
    EXPECT_TRUE(bits.test(5));
    EXPECT_FALSE(bits.test(4));
    EXPECT_FALSE(bits.test(6));

    bits.set(0);
    bits.set(31);
    EXPECT_TRUE(bits.test(0));
    EXPECT_TRUE(bits.test(31));
    EXPECT_TRUE(bits.test(5));
}

TEST(AtomicBitsTest, ClearBit) {
    AtomicBits<32> bits;

    bits.set(10);
    EXPECT_TRUE(bits.test(10));

    bits.clear(10);
    EXPECT_FALSE(bits.test(10));

    // Clearing already cleared bit should not break anything
    bits.clear(10);
    EXPECT_FALSE(bits.test(10));
}

TEST(AtomicBitsTest, TestAndSet) {
    AtomicBits<32> bits;

    // First test_and_set should return false and set the bit
    EXPECT_FALSE(bits.test_and_set(15));
    EXPECT_TRUE(bits.test(15));

    // Second test_and_set should return true and keep bit set
    EXPECT_TRUE(bits.test_and_set(15));
    EXPECT_TRUE(bits.test(15));
}

TEST(AtomicBitsTest, TestAndClear) {
    AtomicBits<32> bits;

    EXPECT_FALSE(bits.test_and_clear(20));
    EXPECT_FALSE(bits.test(20));

    bits.set(20);
    EXPECT_TRUE(bits.test_and_clear(20));
    EXPECT_FALSE(bits.test(20));

    // Second test_and_clear should return false
    EXPECT_FALSE(bits.test_and_clear(20));
}

TEST(AtomicBitsTest, SetAll) {
    AtomicBits<32> bits;

    bits.set_all();

    for (size_t i = 0; i < 32; ++i) {
        EXPECT_TRUE(bits.test(i));
    }
}

TEST(AtomicBitsTest, ClearAll) {
    AtomicBits<32> bits;

    bits.set(5);
    bits.set(10);
    bits.set(25);

    bits.clear_all();

    for (size_t i = 0; i < 32; ++i) {
        EXPECT_FALSE(bits.test(i));
    }
}

TEST(AtomicBitsTest, OutOfBoundsIndices) {
    AtomicBits<32> bits;

    // Out-of-bounds operations should not crash
    bits.set(100);
    bits.clear(200);
    EXPECT_FALSE(bits.test(100));
    EXPECT_FALSE(bits.test_and_set(50));
    EXPECT_FALSE(bits.test_and_clear(75));

    // Valid indices should still work
    bits.set(10);
    EXPECT_TRUE(bits.test(10));
}

TEST(AtomicBitsTest, SmallBitset) {
    AtomicBits<8> bits;

    bits.set(0);
    bits.set(7);
    EXPECT_TRUE(bits.test(0));
    EXPECT_TRUE(bits.test(7));
    EXPECT_FALSE(bits.test(3));

    EXPECT_FALSE(bits.test(8));
}

TEST(AtomicBitsTest, LargeBitset) {
    AtomicBits<100> bits;

    bits.set(0);
    bits.set(50);
    bits.set(99);

    EXPECT_TRUE(bits.test(0));
    EXPECT_TRUE(bits.test(50));
    EXPECT_TRUE(bits.test(99));
    EXPECT_FALSE(bits.test(25));
    EXPECT_FALSE(bits.test(75));
}

TEST(AtomicBitsTest, DifferentStorageTypes) {
    AtomicBits<16, uint16_t> bits16;
    AtomicBits<64, uint64_t> bits64;

    bits16.set(10);
    bits64.set(50);

    EXPECT_TRUE(bits16.test(10));
    EXPECT_TRUE(bits64.test(50));
    EXPECT_FALSE(bits16.test(5));
    EXPECT_FALSE(bits64.test(25));
}

TEST(AtomicBitsTest, MultipleBitsOperations) {
    AtomicBits<64> bits;

    for (size_t i = 0; i < 64; i += 8) {
        bits.set(i);
    }

    for (size_t i = 0; i < 64; ++i) {
        if (i % 8 == 0) {
            EXPECT_TRUE(bits.test(i)) << "Bit " << i;
        } else {
            EXPECT_FALSE(bits.test(i)) << "Bit " << i;
        }
    }

    for (size_t i = 0; i < 32; i += 8) {
        bits.clear(i);
    }

    for (size_t i = 0; i < 32; ++i) {
        EXPECT_FALSE(bits.test(i)) << "Bit " << i;
    }

    for (size_t i = 32; i < 64; i += 8) {
        EXPECT_TRUE(bits.test(i)) << "Bit " << i;
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
