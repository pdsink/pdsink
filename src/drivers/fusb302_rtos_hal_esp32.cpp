#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include <driver/i2c.h>
#include "fusb302_rtos_hal_esp32.h"
#include "fusb302_regs.h"

namespace pd {

namespace fusb302 {

void Fusb302RtosHalEsp32::init_timer() {
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
            self->msg_router->receive(MsgHal_Timer{});
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Fusb302RtosHalEsp32Tick",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, 1000);
}

void Fusb302RtosHalEsp32::init_pins() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << intPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    auto handler = [](void* arg) -> void {
        auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
        self->msg_router->receive(MsgHal_Interrupt{});
    };

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)intPin, handler, this);
}

void Fusb302RtosHalEsp32::start() {
    init_timer();
    init_pins();
}

uint64_t Fusb302RtosHalEsp32::get_timestamp() {
    return esp_timer_get_time() / 1000;
}

bool Fusb302RtosHalEsp32::is_interrupt_active() {
    return gpio_get_level(intPin) == 0; // active low
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