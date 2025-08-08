#pragma once

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_timer.h>
#include "fusb302_regs.h"
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    void set_event_handler(const hal_event_handler_t& handler) override { event_cb = handler; }
    void setup() override;
    uint32_t get_timestamp() override;
    bool read_reg(uint8_t reg, uint8_t& data) override;
    bool write_reg(uint8_t reg, uint8_t data) override;
    bool read_block(uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write_block(uint8_t reg, const uint8_t *data, uint32_t size) override;
    bool is_interrupt_active() override;

    ~Fusb302RtosHalEsp32();

    // Prohibit copy/move because class have interrupt callbacks
    // and manages hardware resources.
    Fusb302RtosHalEsp32(const Fusb302RtosHalEsp32&) = delete;
    Fusb302RtosHalEsp32& operator=(const Fusb302RtosHalEsp32&) = delete;
    Fusb302RtosHalEsp32(Fusb302RtosHalEsp32&&) = delete;
    Fusb302RtosHalEsp32& operator=(Fusb302RtosHalEsp32&&) = delete;
    Fusb302RtosHalEsp32() = default;

protected:
    gpio_num_t sda_io_pin{GPIO_NUM_5};
    gpio_num_t scl_io_pin{GPIO_NUM_6};
    gpio_num_t int_io_pin{GPIO_NUM_7};
    i2c_port_t i2c_num{I2C_NUM_0};
    uint32_t i2c_freq{400000}; // 400kHz
    uint8_t i2c_address{ChipAddress::FUSB302B};

    hal_event_handler_t event_cb;
    esp_timer_handle_t timer_handle;
    bool started{false};

    virtual void init_timer();
    virtual void init_pins();
};

} // namespace fusb302

} // namespace pd
