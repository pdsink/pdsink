#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>
#include "fusb302_rtos.h"
#include "fusb302_regs.h"

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    void set_msg_router(etl::imessage_router& router) { msg_router = &router; };
    void start() override;
    uint64_t get_timestamp() override;
    bool read_reg(uint8_t reg, uint8_t& data) override;
    bool write_reg(uint8_t reg, uint8_t data) override;
    bool read_block(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write_block(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool is_interrupt_active() override;

    ~Fusb302RtosHalEsp32();

protected:
    gpio_num_t sda_io_pin{GPIO_NUM_5};
    gpio_num_t scl_io_pin{GPIO_NUM_6};
    gpio_num_t int_io_pin{GPIO_NUM_7};
    uint8_t i2c_address{ChipAddress::FUSB302B};

    etl::imessage_router* msg_router{nullptr};
    esp_timer_handle_t timer_handle;
    bool started{false};

    virtual void init_timer();
    virtual void init_pins();
};

} // namespace fusb302

} // namespace pd
