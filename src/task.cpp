#include "dpm.h"
#include "pe.h"
#include "prl.h"
#include "task.h"
#include "tc.h"

namespace pd {

void Task::loop() {
    do {
        if (loop_flags.test_and_set(IS_IN_LOOP_FL)) {
            // If processing in progress - postpone call to avoid recursion.
            loop_flags.set(HAS_DEFERRED_FL);
            return;
        }

        auto e_group = event_group.exchange(0);

        if (e_group & Task::EVENT_TIMER_MSK) { port.timers.cleanup(); }

        port.notify_tc(MsgSysUpdate{});

        port.notify_pe(MsgSysUpdate{});
        port.notify_prl(MsgSysUpdate{});

        // Let's rearm timer if needed. 2 cases are possible:
        //
        // 1. start/stop invoked  (in PRL/PE/TC/DPM)
        // 2. Timeout event due timer expire
        //
        // This is NOT needed for periodic 1ms timer without rearm support.

        if (driver->is_rearm_supported()) {
            if (port.timers.timers_changed.exchange(false) ||
                (e_group & Task::EVENT_TIMER_MSK))
            {
                auto next_exp{port.timers.get_next_expiration()};
                if (next_exp != Timers::NO_EXPIRE)
                {
                    if (next_exp == 0) {
                        // Rearm timer event and add deferred call
                        event_group.fetch_or(Task::EVENT_TIMER_MSK);
                        loop_flags.set(HAS_DEFERRED_FL);
                    } else {
                        driver->rearm(next_exp);
                    }
                }
            }
        }

        loop_flags.clear(IS_IN_LOOP_FL);
    } while (loop_flags.test_and_clear(HAS_DEFERRED_FL));
}

void Task::start() {
    loop_flags.set(IS_IN_LOOP_FL);

    port.timers.set_time_provider(
        Timers::GetTimeFunc::create<ITimer, &ITimer::get_timestamp>(*driver)
    );
    port.msgbus.subscribe(task_event_listener);

    driver->start();
    prl->init();
    pe->init();
    tc->start();

    loop_flags.clear(IS_IN_LOOP_FL);
}

void Task_EventListener::on_receive(const MsgTask_Wakeup& msg) {
    task.event_group.fetch_or(Task::EVENT_WAKEUP_MSK);
    task.loop();
}

void Task_EventListener::on_receive(const MsgTask_Timer& msg) {
    task.event_group.fetch_or(Task::EVENT_TIMER_MSK);
    task.loop();
}

} // namespace pd
