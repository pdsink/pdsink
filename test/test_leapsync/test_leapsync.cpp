#include <gtest/gtest.h>
#include "pd/utils/leapsync.h"

TEST(LeapSyncTest, BasicWorkflow) {
    LeapSync<int> sync;
    EXPECT_TRUE(sync.is_idle());

    // Producer enqueues
    sync.enqueue(42);
    EXPECT_FALSE(sync.is_idle());

    // Consumer gets job
    int param;
    EXPECT_TRUE(sync.get_job(param));
    EXPECT_EQ(param, 42);

    // Consumer finishes
    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, VoidParamsWorkflow) {
    LeapSync<> sync;
    EXPECT_TRUE(sync.is_idle());

    sync.enqueue();
    EXPECT_FALSE(sync.is_idle());

    EXPECT_TRUE(sync.get_job());
    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, EnqueueCoalescing) {
    LeapSync<int> sync;

    // Multiple enqueues - only last one matters
    sync.enqueue(100);
    sync.enqueue(200);
    sync.enqueue(300);

    int param;
    EXPECT_TRUE(sync.get_job(param));
    EXPECT_EQ(param, 300);  // Should get the last value

    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, NoJobWhenIdle) {
    LeapSync<int> sync;

    int param;
    EXPECT_FALSE(sync.get_job(param));  // No job when idle
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, NoJobWhenWorking) {
    LeapSync<int> sync;
    sync.enqueue(123);

    int param1, param2;
    EXPECT_TRUE(sync.get_job(param1));   // First get_job succeeds
    EXPECT_FALSE(sync.get_job(param2));  // Second fails - already working

    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, EnqueueDuringWork) {
    LeapSync<int> sync;
    sync.enqueue(111);

    int param;
    EXPECT_TRUE(sync.get_job(param));
    EXPECT_EQ(param, 111);

    // New enqueue while working
    sync.enqueue(222);
    EXPECT_FALSE(sync.is_idle());  // Should be ENQUEUED now

    sync.job_finish();
    // job_finish saw ENQUEUED state, so CAS failed - stays ENQUEUED
    EXPECT_FALSE(sync.is_idle());

    // Can get the new job
    EXPECT_TRUE(sync.get_job(param));
    EXPECT_EQ(param, 222);

    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

TEST(LeapSyncTest, ResetFunctionality) {
    LeapSync<int> sync;
    sync.enqueue(999);

    int param;
    EXPECT_TRUE(sync.get_job(param));
    EXPECT_FALSE(sync.is_idle());  // Working state

    sync.reset();
    EXPECT_TRUE(sync.is_idle());

    // No more jobs after reset
    EXPECT_FALSE(sync.get_job(param));
}

TEST(LeapSyncTest, EnumParams) {
    enum class Mode { FAST, SLOW, AUTO };
    LeapSync<Mode> sync;

    sync.enqueue(Mode::AUTO);

    Mode mode;
    EXPECT_TRUE(sync.get_job(mode));
    EXPECT_EQ(mode, Mode::AUTO);

    sync.job_finish();
    EXPECT_TRUE(sync.is_idle());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
