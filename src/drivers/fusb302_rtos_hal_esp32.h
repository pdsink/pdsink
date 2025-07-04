#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    Fusb302RtosHalEsp32() : intPin{GPIO_NUM_7} {}
    void set_msg_router(etl::imessage_router& router) { msg_router = &router; };
    void start() override;
    uint64_t get_timestamp() override;
    bool read(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool is_interrupt_active() override;

protected:
    gpio_num_t intPin;
    etl::imessage_router* msg_router{nullptr};
    esp_timer_handle_t timer_handle;

    virtual void init_timer();
    virtual void init_pins();
};

} // namespace fusb302

} // namespace pd
