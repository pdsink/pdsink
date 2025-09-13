#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifndef PD_ESP32_USE_NEW_I2C_MASTER_API
#  if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#    define PD_ESP32_USE_NEW_I2C_MASTER_API 1
#  else
#    define PD_ESP32_USE_NEW_I2C_MASTER_API 0
#  endif
#endif

#if PD_ESP32_USE_NEW_I2C_MASTER_API
#  include <driver/i2c_master.h>
#  include <etl/flat_map.h>
#  include <etl/utility.h>
#else
#  include <driver/i2c.h>
#endif
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

class Fusb302RtosHalEsp32 : public IFusb302RtosHal {
public:
    void set_event_handler(const hal_event_handler_t& handler) override { event_cb = handler; }
    void setup() override;
    ITimer::TimeFunc get_time_func() const override;
    bool is_interrupt_active() override;

    // I2C API, can be used from other application modules independently,
    // when the bus is shared between multiple devices.
    virtual void init_i2c();
    bool read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t& data) override;
    bool write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) override;
    bool read_block(uint8_t i2c_addr, uint8_t reg, uint8_t *data, uint32_t size) override;
    bool write_block(uint8_t i2c_addr, uint8_t reg, const uint8_t *data, uint32_t size) override;

    ~Fusb302RtosHalEsp32();

    // Prohibit copy/move because class have interrupt callbacks
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

    hal_event_handler_t event_cb;
    esp_timer_handle_t timer_handle;
    bool started{false};
    bool i2c_initialized{false};

#if PD_ESP32_USE_NEW_I2C_MASTER_API
    i2c_master_bus_handle_t i2c_bus{nullptr};
    // Max total count of devices on I2C bus, if this driver is reused in multiple places.
    // Should be enough for most applications.
    static constexpr size_t MAX_I2C_DEV_CACHE = 5;
    etl::flat_map<uint8_t, i2c_master_dev_handle_t, MAX_I2C_DEV_CACHE> i2c_dev_cache{};
    SemaphoreHandle_t i2c_dev_cache_mutex{nullptr};
    auto ensure_i2c_master_dev(uint8_t addr) -> i2c_master_dev_handle_t;
#endif

    virtual void init_timer();
    virtual void init_fusb_interrupt();
};

} // namespace fusb302

} // namespace pd
