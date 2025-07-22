#include <gtest/gtest.h>
#include "utils/leapsync.h"

TEST(LeapSyncTest, BasicEnquireAndReady) {
    LeapSync<int> sync;
    EXPECT_TRUE(sync.is_ready());

    sync.enquire(42);
    EXPECT_FALSE(sync.is_ready());

    EXPECT_TRUE(sync.is_enquired());
    sync.mark_started();
    EXPECT_TRUE(sync.is_started());
    sync.mark_finished();

    EXPECT_TRUE(sync.is_ready());
    EXPECT_FALSE(sync.is_started());
}

TEST(LeapSyncTest, NoParamsVersion) {
    LeapSync<> sync;
    EXPECT_TRUE(sync.is_ready());

    sync.enquire();
    EXPECT_FALSE(sync.is_ready());

    sync.mark_started();
    sync.mark_finished();

    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, RequestCoalescing) {
    LeapSync<int> sync;
    sync.enquire(100);
    sync.enquire(200);
    sync.enquire(300);

    EXPECT_TRUE(sync.is_enquired());
    sync.mark_started();

    int param = sync.get_param().load();
    EXPECT_EQ(param, 300);

    sync.mark_finished();
    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, WorkerWorkflow) {
    LeapSync<bool> sync;
    EXPECT_FALSE(sync.is_enquired());
    EXPECT_FALSE(sync.is_started());

    sync.enquire(true);
    EXPECT_TRUE(sync.is_enquired());
    EXPECT_FALSE(sync.is_started());

    sync.mark_started();
    EXPECT_FALSE(sync.is_enquired());
    EXPECT_TRUE(sync.is_started());

    sync.mark_finished();
    EXPECT_FALSE(sync.is_enquired());
    EXPECT_FALSE(sync.is_started());
    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, ResetFunctionality) {
    LeapSync<int> sync;
    sync.enquire(555);
    sync.mark_started();
    EXPECT_TRUE(sync.is_started());
    EXPECT_FALSE(sync.is_ready());

    sync.reset();
    EXPECT_FALSE(sync.is_started());
    EXPECT_TRUE(sync.is_ready());
    EXPECT_FALSE(sync.is_enquired());
}

TEST(LeapSyncTest, EnumParams) {
    enum class Mode { FAST, SLOW, AUTO };
    LeapSync<Mode> sync;

    sync.enquire(Mode::AUTO);
    sync.mark_started();

    Mode mode = sync.get_param().load();
    EXPECT_EQ(mode, Mode::AUTO);

    sync.mark_finished();
    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, DoubleMarkStarted) {
    LeapSync<int> sync;
    sync.enquire(123);
    sync.mark_started();
    sync.mark_started();

    EXPECT_TRUE(sync.is_started());

    sync.mark_finished();
    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, CounterOverflow) {
    LeapSync<> sync;
    for (int i = 0; i < 65530; ++i) {
        sync.enquire();
        sync.mark_started();
        sync.mark_finished();
    }

    sync.enquire();
    EXPECT_TRUE(sync.is_enquired());

    sync.mark_started();
    EXPECT_TRUE(sync.is_started());

    sync.mark_finished();
    EXPECT_TRUE(sync.is_ready());
}

TEST(LeapSyncTest, MultipleEnquireDuringWork) {
    LeapSync<uint8_t> sync;
    sync.enquire(10);
    sync.mark_started();

    sync.enquire(20);
    sync.enquire(30);

    EXPECT_TRUE(sync.is_started());
    EXPECT_FALSE(sync.is_ready());

    sync.mark_finished();

    EXPECT_TRUE(sync.is_enquired());
    EXPECT_FALSE(sync.is_ready());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
