#pragma once

#include <etl/atomic.h>
#include <stddef.h>
#include <stdint.h>

#include "data_objects.h"
#include "idriver.h"
#include "messages.h"
#include "port.h"
#include "utils/atomic_bits.h"

namespace pd {

class TC;
class PE;
class IDPM;
class PRL;
class Sink;

using Task_EventListener_Base = etl::message_router<class Task_EventListener,
    MsgTask_Wakeup,
    MsgTask_Timer>;

class Task_EventListener : public Task_EventListener_Base {
public:
    Task_EventListener(Sink& sink) : Task_EventListener_Base(ROUTER_ID::TASK), sink(sink) {}
    void on_receive(const MsgTask_Wakeup& msg);
    void on_receive(const MsgTask_Timer& msg);
    void on_receive_unknown(const etl::imessage& msg) {};
private:
    Sink& sink;
};

class Sink {
public:
    Sink(Port& port) : port{port}, task_event_listener{*this} {}

    // Disable unexpected use
    Sink() = delete;
    Sink(const Sink&) = delete;
    Sink& operator=(const Sink&) = delete;

    void start();
    void loop();

    IDriver* driver{nullptr};
    TC* tc{nullptr};
    PE* pe{nullptr};
    IDPM* dpm{nullptr};
    PRL* prl{nullptr};
    Port& port;

    static constexpr uint32_t EVENT_TIMER_MSK = 1ul << 0;
    static constexpr uint32_t EVENT_WAKEUP_MSK = 1ul << 1;

    etl::atomic<uint32_t> event_group{0};

private:

    enum {
        IS_IN_LOOP_FL,
        HAS_DEFERRED_FL,
        LOOP_FLAGS_COUNT
    };
    AtomicBits<LOOP_FLAGS_COUNT> loop_flags;

    Task_EventListener task_event_listener;
};

} // namespace pd
