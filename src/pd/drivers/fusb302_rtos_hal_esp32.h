#pragma once

#include "fusb302_rtos.h"

#include <etl/atomic.h>
#include <driver/gpio.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define _PD_HAL_USE_RTOS_TICKS (PD_TIMER_TICK_US == 1000)

#if _PD_HAL_USE_RTOS_TICKS
#include <freertos/timers.h>
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/gptimer.h>
#endif

// Use old i2c API because the new one still has a serious bug
// https://github.com/espressif/esp-idf/issues/14030
#include <driver/i2c.h>

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    void set_event_handler(const hal_event_handler_t& handler) override { event_cb.store(handler); }
    void setup() override;
    ITimer::TickSource get_tick_source() const override;
    void rearm(uint32_t deadline_ticks) override;
    bool is_interrupt_active() override;

    // The I2C API can be used by other application modules independently
    // when the bus is shared between multiple devices.
    virtual void init_i2c();
    bool read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t& data) override;
    bool write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) override;
    bool read_block(uint8_t i2c_addr, uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write_block(uint8_t i2c_addr, uint8_t reg, const uint8_t *data, uint32_t size) override;

    ~Fusb302RtosHalEsp32();

    // Prohibit copy/move because the class has interrupt callbacks
    // and manages hardware resources.
    Fusb302RtosHalEsp32(const Fusb302RtosHalEsp32&) = delete;
    Fusb302RtosHalEsp32& operator=(const Fusb302RtosHalEsp32&) = delete;
    Fusb302RtosHalEsp32(Fusb302RtosHalEsp32&&) = delete;
    Fusb302RtosHalEsp32& operator=(Fusb302RtosHalEsp32&&) = delete;
    Fusb302RtosHalEsp32();

protected:
    gpio_num_t sda_io_pin{GPIO_NUM_5};
    gpio_num_t scl_io_pin{GPIO_NUM_6};
    gpio_num_t int_io_pin{GPIO_NUM_7};
    i2c_port_t i2c_num{I2C_NUM_0};
    uint32_t i2c_freq{400000}; // 400kHz

    etl::atomic<hal_event_handler_t> event_cb{hal_event_handler_t{}};
#if _PD_HAL_USE_RTOS_TICKS
    TimerHandle_t timer_handle{nullptr};
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_handle_t timer_handle{nullptr};
#endif
    bool started{false};
    bool i2c_initialized{false};

    uint32_t get_ticks() const;
    virtual void init_timer();
    virtual void init_fusb_interrupt();
};

} // namespace fusb302

} // namespace pd
