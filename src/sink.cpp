#include "dpm.h"
#include "pe.h"
#include "prl.h"
#include "sink.h"
#include "tc.h"

namespace pd {

void Sink::loop() {
    do {
        if (!loop_flags.test_and_set(IS_IN_LOOP_FL)) {
            // If processing in progress - postpone call to avoid recursion.
            loop_flags.set(HAS_DEFERRED_FL);
            return;
        }

        MsgPdEvents msg_events{event_group.exchange(0)};
        if (msg_events.has_timeout()) timers.cleanup();

        tc->dispatch(msg_events);
        auto connected = tc->is_connected();

        dpm->dispatch(msg_events, connected);
        pe->dispatch(msg_events, connected);
        prl->dispatch(msg_events, connected);

        // Let's rearm timer if needed. 2 cases are possible:
        //
        // 1. start/stop invoked  (in PRL/PE/TC/DPM)
        // 2. Timeout event due timer expire
        //
        // This is NOT needed for periodic 1ms timer without rearm support.

        if (driver->is_rearm_supported()) {
            if (timers.timers_changed.exchange(false) ||
                msg_events.has_timeout())
            {
                auto next_exp{timers.get_next_expiration()};
                if (next_exp != Timers::NO_EXPIRE)
                {
                    if (next_exp == 0) {
                        set_event(PD_EVENT::TIMER);
                    } else {
                        driver->rearm(next_exp);
                    }
                }
            }
        }

        loop_flags.clear(IS_IN_LOOP_FL);
    } while (loop_flags.test_and_clear(HAS_DEFERRED_FL));
}

void Sink::start() {
    loop_flags.set(IS_IN_LOOP_FL);

    driver->start();
    prl->init();
    pe->init();
    tc->start();
    //dpm->start();

    loop_flags.clear(IS_IN_LOOP_FL);
    set_event(PD_EVENT::TIMER);
}

} // namespace pd
