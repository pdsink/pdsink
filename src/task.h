#pragma once

#include <etl/atomic.h>
#include <stddef.h>
#include <stdint.h>

#include "messages.h"
#include "utils/atomic_bits.h"

namespace pd {

class Port; class Task; class TC; class IDPM; class PE; class PRL; class IDriver;

using Task_EventListener_Base = etl::message_router<class Task_EventListener,
    MsgTask_Wakeup,
    MsgTask_Timer>;

class Task_EventListener : public Task_EventListener_Base {
public:
    Task_EventListener(Task& task) : Task_EventListener_Base(ROUTER_ID::TASK), task(task) {}
    void on_receive(const MsgTask_Wakeup& msg);
    void on_receive(const MsgTask_Timer& msg);
    void on_receive_unknown(const etl::imessage& msg) {};
private:
    Task& task;
};

class Task {
public:
    Task(Port& port, IDriver& driver) : port{port}, driver{driver}, task_event_listener{*this} {}

    // Disable unexpected use
    Task() = delete;
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    void start(TC& tc, IDPM& dpm, PE& pe, PRL& prl, IDriver& driver);

    void loop();

    static constexpr uint32_t EVENT_TIMER_MSK = 1ul << 0;
    static constexpr uint32_t EVENT_WAKEUP_MSK = 1ul << 1;

    etl::atomic<uint32_t> event_group{0};

private:
    Port& port;
    IDriver& driver;

    enum {
        IS_IN_LOOP_FL,
        HAS_DEFERRED_FL,
        LOOP_FLAGS_COUNT
    };
    AtomicBits<LOOP_FLAGS_COUNT> loop_flags;

    Task_EventListener task_event_listener;
};

} // namespace pd
