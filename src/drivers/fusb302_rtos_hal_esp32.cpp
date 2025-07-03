#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include <driver/i2c.h>
#include "fusb302_rtos_hal_esp32.h"
#include "fusb302_regs.h"

namespace pd {

namespace fusb302 {

void Fusb302RtosHalEsp32::set_tick_handler(etl::delegate<void()> handler) {
    // Stop existing timer
    if (timer_handle != nullptr) {
        esp_timer_stop(timer_handle);
        esp_timer_delete(timer_handle);
        timer_handle = nullptr;
    }

    tick_handler = handler;

    // If handler not empty - create a new timer
    if (tick_handler.is_valid()) {
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
                if (self->tick_handler) {
                    self->tick_handler();
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "Fusb302RtosHalEsp32Tick",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&timer_args, &timer_handle);
        esp_timer_start_periodic(timer_handle, 1000);
    }
}

uint64_t Fusb302RtosHalEsp32::get_timestamp() {
    return esp_timer_get_time() / 1000;
}

static constexpr int I2C_TIMEOUT_MS = 10;
static_assert(pdMS_TO_TICKS(I2C_TIMEOUT_MS) > 0, "Too slow FreeRTOS tick rate, should be 1ms or faster");

bool Fusb302RtosHalEsp32::read(uint8_t reg, uint8_t *data, uint32_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (fusb302::DeviceID::addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (fusb302::DeviceID::addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool Fusb302RtosHalEsp32::write(uint8_t reg, uint8_t *data, uint32_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (fusb302::DeviceID::addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS_HAL_ESP32