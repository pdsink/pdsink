#include "sink.h"
#include "tc.h"
#include "dpm.h"
#include "pe.h"
#include "prl.h"

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

        loop_flags.clear(IS_IN_LOOP_FL);
    } while (loop_flags.test_and_clear(HAS_DEFERRED_FL));
}

void Sink::start() {
    tc->start();
    pe->start();
    //prl->start();
    dpm->start();
    driver->start();
    loop();
}

} // namespace pd
