#include <gtest/gtest.h>
#include <etl/vector.h>
#include <cstring>
#include "pd/utils/spsc_overwrite_queue.h"

// Basic functionality tests
TEST(SPSCOverwriteQueueTest, InitiallyEmpty) {
    spsc_overwrite_queue<int, 4> queue;
    EXPECT_TRUE(queue.empty());

    int value;
    EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCOverwriteQueueTest, PushSingleElement) {
    spsc_overwrite_queue<int, 4> queue;

    queue.push(42);
    EXPECT_FALSE(queue.empty());

    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 42);
    EXPECT_TRUE(queue.empty());
}

TEST(SPSCOverwriteQueueTest, PushPopSequence) {
    spsc_overwrite_queue<int, 4> queue;

    // Fill queue partially
    queue.push(1);
    queue.push(2);
    queue.push(3);
    EXPECT_FALSE(queue.empty());

    // Pop elements in order
    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 1);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 2);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 3);

    EXPECT_TRUE(queue.empty());
    EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCOverwriteQueueTest, FillToCapacity) {
    spsc_overwrite_queue<int, 4> queue;

    // Fill to capacity (4 elements)
    queue.push(10);
    queue.push(20);
    queue.push(30);
    queue.push(40);
    EXPECT_FALSE(queue.empty());

    // Pop all elements
    int value;
    for (int expected = 10; expected <= 40; expected += 10) {
        EXPECT_TRUE(queue.pop(value));
        EXPECT_EQ(value, expected);
    }

    EXPECT_TRUE(queue.empty());
}

// Overwrite behavior tests
TEST(SPSCOverwriteQueueTest, OverwriteOldestWhenFull) {
    spsc_overwrite_queue<int, 4> queue;

    // Fill to capacity
    queue.push(1);
    queue.push(2);
    queue.push(3);
    queue.push(4);

    // Add one more - should overwrite oldest (1)
    queue.push(5);

    // Should get 2, 3, 4, 5
    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 2);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 3);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 4);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 5);

    EXPECT_TRUE(queue.empty());
}

TEST(SPSCOverwriteQueueTest, ContinuousOverwrite) {
    spsc_overwrite_queue<int, 4> queue;

    // Fill beyond capacity multiple times
    for (int i = 1; i <= 10; ++i) {
        queue.push(i);
    }

    // Should have last 4 elements: 7, 8, 9, 10
    int value;
    for (int expected = 7; expected <= 10; ++expected) {
        EXPECT_TRUE(queue.pop(value));
        EXPECT_EQ(value, expected);
    }

    EXPECT_TRUE(queue.empty());
}

// Mixed operations tests
TEST(SPSCOverwriteQueueTest, InterleavedPushPop) {
    spsc_overwrite_queue<int, 4> queue;

    queue.push(1);
    queue.push(2);

    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 1);

    queue.push(3);
    queue.push(4);
    queue.push(5); // This will use the slot freed by popping 1

    // Should have: 2, 3, 4, 5
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 2);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 3);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 4);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 5);

    EXPECT_TRUE(queue.empty());
}

// Clear functionality tests
TEST(SPSCOverwriteQueueTest, ClearFromConsumer) {
    spsc_overwrite_queue<int, 4> queue;

    queue.push(1);
    queue.push(2);
    queue.push(3);
    EXPECT_FALSE(queue.empty());

    queue.clear_from_consumer();
    EXPECT_TRUE(queue.empty());

    int value;
    EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCOverwriteQueueTest, ClearFromProducer) {
    spsc_overwrite_queue<int, 4> queue;

    queue.push(1);
    queue.push(2);
    queue.push(3);
    EXPECT_FALSE(queue.empty());

    queue.clear_from_producer();
    EXPECT_TRUE(queue.empty());

    int value;
    EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCOverwriteQueueTest, OperationsAfterClear) {
    spsc_overwrite_queue<int, 4> queue;

    // Fill and clear
    queue.push(1);
    queue.push(2);
    queue.clear_from_consumer();

    // Should work normally after clear
    queue.push(10);
    queue.push(20);

    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 10);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 20);

    EXPECT_TRUE(queue.empty());
}

// Different types and larger queue tests

TEST(SPSCOverwriteQueueTest, LargerQueue) {
    spsc_overwrite_queue<int, 16> queue;

    // Fill completely
    for (int i = 0; i < 16; ++i) {
        queue.push(i);
    }

    // Add more to test overwrite
    for (int i = 16; i < 20; ++i) {
        queue.push(i);
    }

    // Should have values 4-19 (last 16)
    int value;
    for (int expected = 4; expected < 20; ++expected) {
        EXPECT_TRUE(queue.pop(value));
        EXPECT_EQ(value, expected);
    }

    EXPECT_TRUE(queue.empty());
}

// Edge cases
TEST(SPSCOverwriteQueueTest, MinimumSize) {
    spsc_overwrite_queue<int, 2> queue;

    queue.push(1);
    queue.push(2);

    // Adding third should overwrite first
    queue.push(3);

    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 2);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 3);

    EXPECT_TRUE(queue.empty());
}

TEST(SPSCOverwriteQueueTest, RepeatedClearAndFill) {
    spsc_overwrite_queue<int, 4> queue;

    for (int cycle = 0; cycle < 3; ++cycle) {
        // Fill queue
        for (int i = 0; i < 4; ++i) {
            queue.push(cycle * 10 + i);
        }

        // Clear
        queue.clear_from_consumer();
        EXPECT_TRUE(queue.empty());
    }

    // Final fill and verify
    queue.push(100);
    queue.push(200);

    int value;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 100);

    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 200);

    EXPECT_TRUE(queue.empty());
}

// Test with custom type
struct TestStruct {
    int value;
    etl::vector<char, 16> name;  // Fixed-size vector

    TestStruct() = default;
    TestStruct(int v, const char* n) : value(v) {
        name.assign(n, n + strlen(n));
    }

    bool operator==(const TestStruct& other) const {
        return value == other.value && name == other.name;
    }
};

TEST(SPSCOverwriteQueueTest, CustomType) {
    spsc_overwrite_queue<TestStruct, 4> queue;

    queue.push(TestStruct{1, "first"});
    queue.push(TestStruct{2, "second"});

    TestStruct result;
    EXPECT_TRUE(queue.pop(result));
    EXPECT_EQ(result, TestStruct(1, "first"));

    EXPECT_TRUE(queue.pop(result));
    EXPECT_EQ(result, TestStruct(2, "second"));

    EXPECT_TRUE(queue.empty());
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
