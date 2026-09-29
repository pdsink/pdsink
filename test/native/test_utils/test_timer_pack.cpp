#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "pd/utils/timer_pack.h"

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
    }

    void advance_time(uint32_t delta) {
        current_time += delta;
    }

    void set_time(uint32_t time) {
        current_time = time;
    }

    TimerPack<TEST_TIMER_COUNT> timers;
    uint32_t current_time;
};

TEST_F(TimerPackTest, BasicStartStop) {
    const int timer_id = TIMER_0;
    const uint32_t timeout = 100;

    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    timers.start(timer_id, current_time + timeout);
    EXPECT_FALSE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    advance_time(timeout - 1);
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id, current_time));
    EXPECT_FALSE(timers.is_disabled(timer_id));

    timers.stop(timer_id);
    EXPECT_TRUE(timers.is_disabled(timer_id));
}

TEST_F(TimerPackTest, TimeOverflow) {
    const int timer_id = TIMER_0;
    const uint32_t timeout = 200;

    set_time(UINT32_MAX - 100);
    timers.start(timer_id, current_time + timeout);

    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    // Cross overflow: UINT32_MAX - 100 + 200 = 99 after overflow
    advance_time(150);
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(timer_id, current_time));
}

TEST_F(TimerPackTest, MultipleTimers) {
    timers.start(TIMER_0, current_time + 100);
    timers.start(TIMER_1, current_time + 200);
    timers.start(TIMER_2, current_time + 50);

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(TIMER_2, current_time));
    EXPECT_FALSE(timers.is_expired(TIMER_0, current_time));
    EXPECT_FALSE(timers.is_expired(TIMER_1, current_time));

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(TIMER_0, current_time));
    EXPECT_FALSE(timers.is_expired(TIMER_1, current_time));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(TIMER_1, current_time));
}

TEST_F(TimerPackTest, Cleanup) {
    timers.start(TIMER_0, current_time + 100);
    timers.start(TIMER_1, current_time + 200);

    advance_time(150);

    auto next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 1100U);

    timers.cleanup(current_time);

    EXPECT_TRUE(timers.is_expired(TIMER_0, current_time));
    EXPECT_FALSE(timers.is_expired(TIMER_1, current_time));
}

TEST_F(TimerPackTest, NextDeadline) {
    EXPECT_FALSE(timers.get_next_deadline(current_time));

    timers.start(TIMER_0, current_time + 100);
    auto next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 1100U);

    advance_time(50);
    next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 1100U);

    timers.start(TIMER_1, current_time + 20);
    next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 1070U);

    advance_time(60);
    next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 1070U);

    timers.stop(TIMER_0);
    timers.stop(TIMER_1);
    set_time(UINT32_MAX - 10);
    timers.start(TIMER_0, UINT32_MAX);
    timers.start(TIMER_1, 0);

    next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, UINT32_MAX);

    advance_time(10);
    timers.cleanup(current_time);
    next = timers.get_next_deadline(current_time);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 0U);

    advance_time(1);
    timers.cleanup(current_time);
    EXPECT_FALSE(timers.get_next_deadline(current_time));
}

TEST_F(TimerPackTest, StopRange) {
    timers.start(TIMER_1, current_time + 100);
    timers.start(TIMER_2, current_time + 200);
    timers.start(TIMER_3, current_time + 300);
    timers.start(TIMER_4, current_time + 400);

    timers.start(TIMER_0, current_time + 500);

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

    timers.start(timer_id, current_time + 100);
    advance_time(50);

    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    timers.start(timer_id, current_time + 200);
    advance_time(100);

    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(timer_id, current_time));
}

TEST_F(TimerPackTest, TimerStates) {
    const int timer_id = TIMER_0;

    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    timers.start(timer_id, current_time + 100);
    EXPECT_FALSE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));

    timers.stop(timer_id);
    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));
}

TEST_F(TimerPackTest, TimersChangedFlag) {
    EXPECT_FALSE(timers.timers_changed.load());

    timers.start(TIMER_0, current_time + 100);
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

    timers.start(timer_id, current_time + 1);
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));
    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id, current_time));

    timers.start(timer_id, current_time + INT32_MAX);
    advance_time(INT32_MAX - 1);
    EXPECT_FALSE(timers.is_expired(timer_id, current_time));
    advance_time(1);
    EXPECT_TRUE(timers.is_expired(timer_id, current_time));
}
