#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "utils/timer_pack.h"

using namespace pd;

constexpr int TIMER_0 = 0;
constexpr int TIMER_1 = 1;
constexpr int TIMER_2 = 2;
constexpr int TIMER_3 = 3;
constexpr int TIMER_4 = 4;
constexpr int TIMER_5 = 5;

constexpr size_t TEST_TIMER_COUNT = 10;

class TimerPackTest : public ::testing::Test {
protected:
    void SetUp() override {
        current_time = 1000;
        timers.set_time(current_time);
    }

    void advance_time(uint32_t delta) {
        current_time += delta;
        timers.set_time(current_time);
    }

    void set_time(uint32_t time) {
        current_time = time;
        timers.set_time(current_time);
    }

    TimerPack<TEST_TIMER_COUNT> timers;
    uint32_t current_time;
};

TEST_F(TimerPackTest, BasicStartStop) {
    const int timer_id = TIMER_0;
    const uint32_t timeout = 100;

    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.start(timer_id, timeout);
    EXPECT_FALSE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    advance_time(timeout - 1);
    EXPECT_FALSE(timers.is_expired(timer_id));

    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id));
    EXPECT_FALSE(timers.is_disabled(timer_id));

    timers.stop(timer_id);
    EXPECT_TRUE(timers.is_disabled(timer_id));
}

TEST_F(TimerPackTest, TimeOverflow) {
    const int timer_id = TIMER_0;
    const uint32_t timeout = 200;

    set_time(UINT32_MAX - 100);
    timers.start(timer_id, timeout);

    EXPECT_FALSE(timers.is_expired(timer_id));

    // Cross overflow: UINT32_MAX - 100 + 200 = 99 after overflow
    advance_time(150);
    EXPECT_FALSE(timers.is_expired(timer_id));

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(timer_id));
}

TEST_F(TimerPackTest, MultipleTimers) {
    timers.start(TIMER_0, 100);
    timers.start(TIMER_1, 200);
    timers.start(TIMER_2, 50);

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(TIMER_2));
    EXPECT_FALSE(timers.is_expired(TIMER_0));
    EXPECT_FALSE(timers.is_expired(TIMER_1));

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(TIMER_0));
    EXPECT_FALSE(timers.is_expired(TIMER_1));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(TIMER_1));
}

TEST_F(TimerPackTest, Cleanup) {
    timers.start(TIMER_0, 100);
    timers.start(TIMER_1, 200);

    advance_time(150);

    EXPECT_TRUE(timers.is_expired(TIMER_0));

    timers.cleanup();

    EXPECT_TRUE(timers.is_expired(TIMER_0));
    EXPECT_FALSE(timers.is_expired(TIMER_1));
}

TEST_F(TimerPackTest, NextExpiration) {
    EXPECT_EQ(timers.get_next_expiration(), TimerPack<TEST_TIMER_COUNT>::NO_EXPIRE);

    timers.start(TIMER_0, 100);
    EXPECT_EQ(timers.get_next_expiration(), 100);

    advance_time(50);
    EXPECT_EQ(timers.get_next_expiration(), 50);

    timers.start(TIMER_1, 20);
    EXPECT_EQ(timers.get_next_expiration(), 20);

    advance_time(60);
    EXPECT_EQ(timers.get_next_expiration(), 0);
}

TEST_F(TimerPackTest, StopRange) {
    timers.start(TIMER_1, 100);
    timers.start(TIMER_2, 200);
    timers.start(TIMER_3, 300);
    timers.start(TIMER_4, 400);

    timers.start(TIMER_0, 500);

    EXPECT_FALSE(timers.is_disabled(TIMER_1));
    EXPECT_FALSE(timers.is_disabled(TIMER_0));

    timers.stop_range(TIMER_1, TIMER_4);

    EXPECT_TRUE(timers.is_disabled(TIMER_1));
    EXPECT_TRUE(timers.is_disabled(TIMER_2));
    EXPECT_TRUE(timers.is_disabled(TIMER_3));
    EXPECT_TRUE(timers.is_disabled(TIMER_4));

    EXPECT_FALSE(timers.is_disabled(TIMER_0));
}

TEST_F(TimerPackTest, RestartTimer) {
    const int timer_id = TIMER_0;

    timers.start(timer_id, 100);
    advance_time(50);

    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.start(timer_id, 200);
    advance_time(100);

    EXPECT_FALSE(timers.is_expired(timer_id));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(timer_id));
}

TEST_F(TimerPackTest, TimerStates) {
    const int timer_id = TIMER_0;

    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.start(timer_id, 100);
    EXPECT_FALSE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.stop(timer_id);
    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));
}

TEST_F(TimerPackTest, TimersChangedFlag) {
    EXPECT_FALSE(timers.timers_changed.load());

    timers.start(TIMER_0, 100);
    EXPECT_TRUE(timers.timers_changed.load());

    timers.timers_changed.store(false);
    timers.stop(TIMER_0);
    EXPECT_TRUE(timers.timers_changed.load());

    timers.timers_changed.store(false);
    timers.stop_range(TIMER_1, TIMER_4);
    EXPECT_TRUE(timers.timers_changed.load());
}

TEST_F(TimerPackTest, EdgeTimeValues) {
    const int timer_id = TIMER_0;

    timers.start(timer_id, 1);
    EXPECT_FALSE(timers.is_expired(timer_id));
    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id));

    timers.start(timer_id, UINT32_MAX);
    advance_time(UINT32_MAX - 1);
    EXPECT_FALSE(timers.is_expired(timer_id));
    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
