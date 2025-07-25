#pragma once

#include <etl/atomic.h>
#include <stddef.h>
#include <stdint.h>

#include "data_objects.h"
#include "idriver.h"
#include "port.h"
#include "timers.h"
#include "utils/atomic_bits.h"

namespace pd {

class TC;
class PE;
class IDPM;
class PRL;

class Sink {
public:
    Sink(Port& port) : port{port} {}

    // Disable unexpected use
    Sink() = delete;
    Sink(const Sink&) = delete;
    Sink& operator=(const Sink&) = delete;

    void start();

    void wakeup() { set_event(PD_EVENT::WAKEUP); };
    void set_event(uint32_t event) {
        event_group.fetch_or(event);
        loop();
    };

    Timers timers{};
    IDriver* driver{nullptr};
    TC* tc{nullptr};
    PE* pe{nullptr};
    IDPM* dpm{nullptr};
    PRL* prl{nullptr};
    Port& port;

private:
    void loop();

    enum {
        IS_IN_LOOP_FL,
        HAS_DEFERRED_FL,
        LOOP_FLAGS_COUNT
    };
    AtomicBits<LOOP_FLAGS_COUNT> loop_flags;

    etl::atomic<uint32_t> event_group{0};
};

} // namespace pd
