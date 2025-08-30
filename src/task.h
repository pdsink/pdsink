#pragma once

#include <etl/atomic.h>
#include <stddef.h>
#include <stdint.h>

#include "messages.h"
#include "utils/atomic_enum_bits.h"

namespace pd {

class Port; class Task; class TC; class IDPM; class PE; class PRL; class IDriver;

using Task_EventListener_Base = etl::message_router<class Task_EventListener,
    MsgTask_Wakeup,
    MsgTask_Timer>;

class Task_EventListener : public Task_EventListener_Base {
public:
    Task_EventListener(Task& task) : task(task) {}
    void on_receive(const MsgTask_Wakeup& msg);
    void on_receive(const MsgTask_Timer& msg);
    void on_receive_unknown(const etl::imessage&);
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

    virtual void start(TC& tc, IDPM& dpm, PE& pe, PRL& prl, IDriver& driver);
    virtual void set_event(uint32_t event_mask);
    virtual void dispatch();
    virtual void tick();

    static constexpr uint32_t EVENT_TIMER_MSK = 1ul << 0;
    static constexpr uint32_t EVENT_WAKEUP_MSK = 1ul << 1;

protected:
    Port& port;
    IDriver& driver;
    etl::atomic<uint32_t> event_group{0};

    enum class GUARD_FLAGS {
        IS_IN_TICK,
        HAS_DEFERRED_CALL,
        _Count
    };
    AtomicEnumBits<GUARD_FLAGS> tick_guard_flags{};

    Task_EventListener task_event_listener;
};

} // namespace pd
