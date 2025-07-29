#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "timers.h"

using namespace pd;

class TimersTest : public ::testing::Test {
protected:
    void SetUp() override {
        current_time = 1000;

        timers.set_time_provider(
            etl::delegate<PD_TIME_T()>::create<TimersTest, &TimersTest::get_current_time>(*this)
        );
    }

    PD_TIME_T get_current_time() { return current_time; }

    void advance_time(uint32_t delta) { current_time += delta; }

    void set_time(PD_TIME_T time) { current_time = time; }

    Timers timers;
    PD_TIME_T current_time;
};

TEST_F(TimersTest, BasicStartStop) {
    const int timer_id = PD_TIMER::TC_DEBOUNCE;
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

TEST_F(TimersTest, TimeoutTypeInterface) {
    timers.start(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
    EXPECT_FALSE(timers.is_disabled(PD_TIMEOUT::TC_VBUS_DEBOUNCE));
    EXPECT_FALSE(timers.is_expired(PD_TIMEOUT::TC_VBUS_DEBOUNCE));

    timers.stop(PD_TIMEOUT::TC_VBUS_DEBOUNCE);
    EXPECT_TRUE(timers.is_disabled(PD_TIMEOUT::TC_VBUS_DEBOUNCE));
}

TEST_F(TimersTest, TimeOverflow) {
    const int timer_id = PD_TIMER::TC_DEBOUNCE;
    const uint32_t timeout = 200;

    set_time(UINT32_MAX - 100);
    timers.start(timer_id, timeout);

    EXPECT_FALSE(timers.is_expired(timer_id));

    // Cross overflow: UINT32_MAX - 100 + 200 = 99 after overflow
    set_time(50);
    EXPECT_FALSE(timers.is_expired(timer_id));

    set_time(100);
    EXPECT_TRUE(timers.is_expired(timer_id));
}

TEST_F(TimersTest, MultipleTimers) {
    timers.start(PD_TIMER::TC_DEBOUNCE, 100);
    timers.start(PD_TIMER::PE_SinkWaitCapTimer, 200);
    timers.start(PD_TIMER::PRL_HardResetCompleteTimer, 50);

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(PD_TIMER::PRL_HardResetCompleteTimer));
    EXPECT_FALSE(timers.is_expired(PD_TIMER::TC_DEBOUNCE));
    EXPECT_FALSE(timers.is_expired(PD_TIMER::PE_SinkWaitCapTimer));

    advance_time(50);
    EXPECT_TRUE(timers.is_expired(PD_TIMER::TC_DEBOUNCE));
    EXPECT_FALSE(timers.is_expired(PD_TIMER::PE_SinkWaitCapTimer));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(PD_TIMER::PE_SinkWaitCapTimer));
}

TEST_F(TimersTest, Cleanup) {
    timers.start(PD_TIMER::TC_DEBOUNCE, 100);
    timers.start(PD_TIMER::PE_SinkWaitCapTimer, 200);

    advance_time(150);

    EXPECT_TRUE(timers.is_expired(PD_TIMER::TC_DEBOUNCE));

    timers.cleanup();

    EXPECT_TRUE(timers.is_expired(PD_TIMER::TC_DEBOUNCE));
    EXPECT_FALSE(timers.is_expired(PD_TIMER::PE_SinkWaitCapTimer));
}

TEST_F(TimersTest, NextExpiration) {
    EXPECT_EQ(timers.get_next_expiration(), Timers::NO_EXPIRE);

    timers.start(PD_TIMER::TC_DEBOUNCE, 100);
    EXPECT_EQ(timers.get_next_expiration(), 100);

    advance_time(50);
    EXPECT_EQ(timers.get_next_expiration(), 50);

    timers.start(PD_TIMER::PE_SinkWaitCapTimer, 20);
    EXPECT_EQ(timers.get_next_expiration(), 20);

    advance_time(60);
    EXPECT_EQ(timers.get_next_expiration(), 0);
}

TEST_F(TimersTest, StopRange) {
    timers.start(PD_TIMER::PE_SinkWaitCapTimer, 100);
    timers.start(PD_TIMER::PE_SenderResponseTimer, 200);
    timers.start(PD_TIMER::PE_SinkRequestTimer, 300);
    timers.start(PD_TIMER::PE_BISTContModeTimer, 400);

    timers.start(PD_TIMER::TC_DEBOUNCE, 500);

    EXPECT_FALSE(timers.is_disabled(PD_TIMER::PE_SinkWaitCapTimer));
    EXPECT_FALSE(timers.is_disabled(PD_TIMER::TC_DEBOUNCE));

    timers.stop_range(PD_TIMERS_RANGE::PE);

    EXPECT_TRUE(timers.is_disabled(PD_TIMER::PE_SinkWaitCapTimer));
    EXPECT_TRUE(timers.is_disabled(PD_TIMER::PE_SenderResponseTimer));
    EXPECT_TRUE(timers.is_disabled(PD_TIMER::PE_SinkRequestTimer));
    EXPECT_TRUE(timers.is_disabled(PD_TIMER::PE_BISTContModeTimer));

    EXPECT_FALSE(timers.is_disabled(PD_TIMER::TC_DEBOUNCE));
}

TEST_F(TimersTest, RestartTimer) {
    const int timer_id = PD_TIMER::TC_DEBOUNCE;

    timers.start(timer_id, 100);
    advance_time(50);

    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.start(timer_id, 200);
    advance_time(100);

    EXPECT_FALSE(timers.is_expired(timer_id));

    advance_time(100);
    EXPECT_TRUE(timers.is_expired(timer_id));
}

TEST_F(TimersTest, TimerStates) {
    const int timer_id = PD_TIMER::TC_DEBOUNCE;

    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.start(timer_id, 100);
    EXPECT_FALSE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));

    timers.stop(timer_id);
    EXPECT_TRUE(timers.is_disabled(timer_id));
    EXPECT_FALSE(timers.is_expired(timer_id));
}

TEST_F(TimersTest, TimersChangedFlag) {
    EXPECT_FALSE(timers.timers_changed.load());

    timers.start(PD_TIMER::TC_DEBOUNCE, 100);
    EXPECT_TRUE(timers.timers_changed.load());

    timers.timers_changed.store(false);
    timers.stop(PD_TIMER::TC_DEBOUNCE);
    EXPECT_TRUE(timers.timers_changed.load());

    timers.timers_changed.store(false);
    timers.stop_range(PD_TIMERS_RANGE::PE);
    EXPECT_TRUE(timers.timers_changed.load());
}

TEST_F(TimersTest, EdgeTimeValues) {
    const int timer_id = PD_TIMER::TC_DEBOUNCE;

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
