#include "common_macros.h"
#include "dpm.h"
#include "idriver.h"
#include "pd_log.h"
#include "pe.h"
#include "port.h"
#include "prl.h"
#include "task.h"
#include "tc.h"

namespace pd {

void Task::tick() {
    auto e_group = event_group.exchange(0);

    // Proceed only if any event available. This check may be useful
    // for manual loop polling.
    if (e_group) {
        if (e_group & Task::EVENT_TIMER_MSK) {
            // Timers don't interact with the system directly. We update
            // internal timestamp value in 2 cases:
            //
            // - when timer event comes
            // - when `.start()` invoked
            //
            // The rest operations can use "old" value safe.
            port.timers.set_time(port.timers.get_time());

            port.timers.cleanup();
        }

        port.notify_tc(MsgSysUpdate{});

        port.notify_pe(MsgSysUpdate{});
        port.notify_prl(MsgSysUpdate{});

        // Let's rearm timer if needed. 2 cases are possible:
        //
        // 1. start/stop invoked  (in PRL/PE/TC/DPM)
        // 2. Timeout event due timer expire
        //
        // This is NOT needed for periodic 1ms timer without rearm support.

        if (driver.is_rearm_supported()) {
            if (port.timers.timers_changed.exchange(false) ||
                (e_group & Task::EVENT_TIMER_MSK))
            {
                auto next_exp{port.timers.get_next_expiration()};
                if (next_exp != Timers::NO_EXPIRE)
                {
                    if (next_exp == 0) {
                        // Rearm timer event and add deferred call
                        event_group.fetch_or(Task::EVENT_TIMER_MSK);
                        tick_guard_flags.set(GUARD_FLAGS::HAS_DEFERRED_CALL);
                    } else {
                        driver.rearm(next_exp);
                    }
                }
            }
        }
    }
}

void Task::dispatch() {
    do {
        // `loop()` can be called again while it is already processing events
        // (e.g. via `set_event`). The `IS_IN_TICK` flag acts as a reentrancy
        // guard: if it is set, we postpone the call by setting `HAS_DEFERRED_CALL`
        // and return immediately. The outer loop at the end of the function
        // checks `HAS_DEFERRED_CALL` and reruns `loop()` to handle the deferred
        // events once the current iteration completes.
        if (tick_guard_flags.test_and_set(GUARD_FLAGS::IS_IN_TICK)) {
            tick_guard_flags.set(GUARD_FLAGS::HAS_DEFERRED_CALL);
            return;
        }

        tick();

        tick_guard_flags.clear(GUARD_FLAGS::IS_IN_TICK);
    } while (tick_guard_flags.test_and_clear(GUARD_FLAGS::HAS_DEFERRED_CALL));
}

void Task::start(TC& tc, IDPM& dpm, PE& pe, PRL& prl, IDriver& drv){
    tick_guard_flags.set(GUARD_FLAGS::IS_IN_TICK);

    port.timers.set_time_provider(drv.get_time_func());
    port.task_rtr = &task_event_listener;
    drv.setup();
    prl.setup();
    pe.setup();
    dpm.setup();
    tc.setup();

    tick_guard_flags.clear(GUARD_FLAGS::IS_IN_TICK);
}

void Task::set_event(uint32_t event_mask) {
    event_group.fetch_or(event_mask);
    dispatch();
}

void Task_EventListener::on_receive(const MsgTask_Wakeup&) {
    task.set_event(Task::EVENT_WAKEUP_MSK);
}

void Task_EventListener::on_receive(const MsgTask_Timer&) {
    task.set_event(Task::EVENT_TIMER_MSK);
}

void Task_EventListener::on_receive_unknown(__maybe_unused const etl::imessage& msg) {
    TASK_LOGE("Task unknown message, ID: {}", msg.get_message_id());
}

} // namespace pd
