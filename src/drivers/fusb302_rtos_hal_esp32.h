#pragma once

#include <esp_timer.h>
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    void set_tick_handler(etl::delegate<void()> handler) override;
    uint64_t get_timestamp() override;
    bool read(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write(uint8_t reg, uint8_t *data, uint32_t size) override;

private:
    esp_timer_handle_t timer_handle = nullptr;
    etl::delegate<void()> tick_handler;
};

} // namespace fusb302

} // namespace pd
