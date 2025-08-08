#include "dpm.h"
#include "idriver.h"
#include "pe.h"
#include "port.h"
#include "prl.h"
#include "task.h"
#include "tc.h"

namespace pd {

void Task::loop() {
    do {
        if (loop_flags.test_and_set(LOOP_FLAGS::IS_IN_LOOP)) {
            // If processing in progress - postpone call to avoid recursion.
            loop_flags.set(LOOP_FLAGS::HAS_DEFERRED);
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
                        loop_flags.set(LOOP_FLAGS::HAS_DEFERRED);
                    } else {
                        driver.rearm(next_exp);
                    }
                }
            }
        }

        loop_flags.clear(LOOP_FLAGS::IS_IN_LOOP);
    } while (loop_flags.test_and_clear(LOOP_FLAGS::HAS_DEFERRED));
}

void Task::start(TC& tc, IDPM& dpm, PE& pe, PRL& prl, IDriver& driver){
    loop_flags.set(LOOP_FLAGS::IS_IN_LOOP);

    port.timers.set_time_provider(driver.get_time_func());
    port.msgbus.subscribe(task_event_listener);
    driver.setup();
    prl.setup();
    pe.setup();
    dpm.setup();
    tc.setup();

    loop_flags.clear(LOOP_FLAGS::IS_IN_LOOP);
}

void Task::set_event(uint32_t event_mask) {
    event_group.fetch_or(event_mask);
    loop();
}

void Task_EventListener::on_receive(const MsgTask_Wakeup&) {
    task.set_event(Task::EVENT_WAKEUP_MSK);
    task.loop();
}

void Task_EventListener::on_receive(const MsgTask_Timer&) {
    task.set_event(Task::EVENT_TIMER_MSK);
    task.loop();
}

} // namespace pd
