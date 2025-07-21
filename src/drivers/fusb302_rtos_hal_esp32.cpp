#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include "fusb302_rtos_hal_esp32.h"

namespace pd {

namespace fusb302 {

void Fusb302RtosHalEsp32::init_timer() {
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
            if (self->msg_router == nullptr) { return; }
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
    // i2c
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_pin;
    conf.scl_io_num = scl_io_pin;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = i2c_freq;

    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));

    // fusb302 interrupt pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << int_io_pin);

    gpio_config(&io_conf);

    auto handler = [](void* arg) -> void {
        auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
        if (self->msg_router == nullptr) { return; }
        self->msg_router->receive(MsgHal_Interrupt{});
    };

    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(gpio_isr_handler_add(int_io_pin, handler, this));
}

void Fusb302RtosHalEsp32::start() {
    if (started) { return; }
    started = true;

    init_timer();
    init_pins();
}

// This will not be actually used, object expected to be static.
Fusb302RtosHalEsp32::~Fusb302RtosHalEsp32() {
    if (started) {
        esp_timer_stop(timer_handle);
        esp_timer_delete(timer_handle);
        gpio_isr_handler_remove(int_io_pin);
        i2c_driver_delete(I2C_NUM_0);
    }
}

uint64_t Fusb302RtosHalEsp32::get_timestamp() {
    return esp_timer_get_time() / 1000;
}

bool Fusb302RtosHalEsp32::is_interrupt_active() {
    return gpio_get_level(int_io_pin) == 0; // active low
}

static constexpr int I2C_TIMEOUT_MS = 10;
static_assert(pdMS_TO_TICKS(I2C_TIMEOUT_MS) > 0, "Too slow FreeRTOS tick rate, should be 1ms or faster");

bool Fusb302RtosHalEsp32::read_block(uint8_t reg, uint8_t *data, uint32_t size) {
    if (!started) { return false; }
    if (!size) { return true; } // nothing to read

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool Fusb302RtosHalEsp32::write_block(uint8_t reg, uint8_t *data, uint32_t size) {
    if (!started) { return false; }
    if (!size) { return true; } // nothing to write

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool Fusb302RtosHalEsp32::read_reg(uint8_t reg, uint8_t& data) {
    return read_block(reg, &data, 1);
}

bool Fusb302RtosHalEsp32::write_reg(uint8_t reg, uint8_t data) {
    return write_block(reg, &data, 1);
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS_HAL_ESP32