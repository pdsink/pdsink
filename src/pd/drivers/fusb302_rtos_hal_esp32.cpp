#include "../pd_conf.h"
#include "../pd_log.h"

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include "fusb302_rtos_hal_esp32.h"

#include "esp_idf_version.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
#error "ESP‑IDF ≥ 4.4.0 required"
#endif

namespace pd {

namespace fusb302 {

static uint32_t get_timestamp() {
    // Alternate implementation:
    // - `esp_timer_get_time() / 1000` (64 bits)
    // - `esp_log_timestamp()`
    // - `millis()` (arduino)

    // FreeRTOS usage is more portable, fast and sleep-friendly.
    // 1ms tick or faster is recommended for correct operation.
    TickType_t t = xPortInIsrContext() ? xTaskGetTickCountFromISR() : xTaskGetTickCount();

    return pdTICKS_TO_MS(t); // (uint32_t)((uint64_t)t * 1000ULL / configTICK_RATE_HZ)
}

void Fusb302RtosHalEsp32::init_i2c() {
    if (i2c_initialized) { return; }
    i2c_initialized = true;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_pin;
    conf.scl_io_num = scl_io_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = i2c_freq;

    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));

    esp_err_t err = i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // Ignore error, if someone already called i2c_driver_install,
        // this is expected use case. But report all other errors.
        ESP_ERROR_CHECK(err);
    }

}

void Fusb302RtosHalEsp32::init_timer() {
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
            if (self->event_cb) {
                self->event_cb(HAL_EVENT_TYPE::Timer, false);
            }
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Fusb302RtosHalEsp32Tick",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000)); // 1ms tick
}

void Fusb302RtosHalEsp32::init_fusb_interrupt() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << int_io_pin);

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    auto handler = [](void* arg) -> void {
        auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
        if (self->event_cb) {
            self->event_cb(HAL_EVENT_TYPE::FUSB302_Interrupt, true);
        }
    };

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // Ignore error, if someone already called gpio_install_isr_service,
        // this is expected use case. But report all other errors.
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(int_io_pin, handler, this));
}

void Fusb302RtosHalEsp32::setup() {
    DRV_LOGI("Fusb302HalEsp32: Starting HAL");
    if (started) { return; }
    started = true;

    init_i2c();
    init_timer();
    init_fusb_interrupt();

    DRV_LOGI("Fusb302HalEsp32: HAL started");
}

// This will not be actually used, object expected to be static.
Fusb302RtosHalEsp32::~Fusb302RtosHalEsp32() {
    if (started) {
        esp_timer_stop(timer_handle);
        esp_timer_delete(timer_handle);
        gpio_isr_handler_remove(int_io_pin);
        i2c_driver_delete(i2c_num);
    }
}

auto Fusb302RtosHalEsp32::get_time_func() const -> ITimer::TimeFunc {
    return get_timestamp;
}

bool Fusb302RtosHalEsp32::is_interrupt_active() {
    return gpio_get_level(int_io_pin) == 0; // active low
}

// I2C timeout: ~3 ms TX + ~6 ms worst‑case BT slot + margin ⇒ 20 ms
static constexpr int I2C_TIMEOUT_MS = 20;
static_assert(pdMS_TO_TICKS(I2C_TIMEOUT_MS) > 0, "Too slow FreeRTOS tick rate, should be 1000Hz or faster");

bool Fusb302RtosHalEsp32::read_block(uint8_t i2c_addr, uint8_t reg, uint8_t *data, uint32_t size) {
    if (!started) { return false; }
    if (!size) { return true; } // nothing to read

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        DRV_LOGE("Fusb302HalEsp32: i2c_cmd_link_create failed");
        return false;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool Fusb302RtosHalEsp32::write_block(uint8_t i2c_addr, uint8_t reg, const uint8_t *data, uint32_t size) {
    if (!started) { return false; }
    if (!size) { return true; } // nothing to write

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        DRV_LOGE("Fusb302HalEsp32: i2c_cmd_link_create failed");
        return false;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool Fusb302RtosHalEsp32::read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t& data) {
    return read_block(i2c_addr, reg, &data, 1);
}

bool Fusb302RtosHalEsp32::write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) {
    return write_block(i2c_addr, reg, &data, 1);
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS_HAL_ESP32
