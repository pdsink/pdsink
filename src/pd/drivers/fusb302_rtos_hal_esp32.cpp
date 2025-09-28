#include "../pd_conf.h"
#include "../pd_log.h"
#include <etl/algorithm.h>

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include "fusb302_rtos_hal_esp32.h"

#include "esp_idf_version.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
#error "ESP‑IDF ≥ 4.4.0 required"
#endif

namespace pd {

namespace fusb302 {

Fusb302RtosHalEsp32::Fusb302RtosHalEsp32() {
#if PD_ESP32_USE_NEW_I2C_MASTER_API
    i2c_dev_cache_mutex = xSemaphoreCreateMutex();
    configASSERT(i2c_dev_cache_mutex != nullptr);
#endif
}

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

#if PD_ESP32_USE_NEW_I2C_MASTER_API
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = i2c_num;
    cfg.sda_io_num = sda_io_pin;
    cfg.scl_io_num = scl_io_pin;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = 1;

    esp_err_t err = i2c_new_master_bus(&cfg, &i2c_bus);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // If the bus is already created elsewhere, report other errors.
        ESP_ERROR_CHECK(err);
    }
#else
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
#endif

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

#if PD_ESP32_USE_NEW_I2C_MASTER_API
        // Remove cached devices first
        xSemaphoreTake(i2c_dev_cache_mutex, portMAX_DELAY);
        for (auto& kv : i2c_dev_cache) {
            if (kv.second) {
                i2c_master_bus_rm_device(kv.second);
            }
        }
        i2c_dev_cache.clear();
        xSemaphoreGive(i2c_dev_cache_mutex);
        if (i2c_bus) {
            i2c_del_master_bus(i2c_bus);
            i2c_bus = nullptr;
        }
#else
        i2c_driver_delete(i2c_num);
#endif
    }

#if PD_ESP32_USE_NEW_I2C_MASTER_API
    vSemaphoreDelete(i2c_dev_cache_mutex);
    i2c_dev_cache_mutex = nullptr;
#endif
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
    if (!i2c_initialized) { return false; }
    if (!size) { return true; } // nothing to read

#if PD_ESP32_USE_NEW_I2C_MASTER_API
    // New I2C master driver (ESP-IDF ≥ 5.1)
    if (i2c_bus == nullptr) { return false; }
    i2c_master_dev_handle_t dev = ensure_i2c_master_dev(i2c_addr);
    if (!dev) { return false; }

    uint8_t reg_byte = reg;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg_byte, 1, data, size, I2C_TIMEOUT_MS);
    return err == ESP_OK;
#else
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
#endif
}

bool Fusb302RtosHalEsp32::write_block(uint8_t i2c_addr, uint8_t reg, const uint8_t *data, uint32_t size) {
    if (!i2c_initialized) { return false; }
    if (!size) { return true; } // nothing to write

#if PD_ESP32_USE_NEW_I2C_MASTER_API
    if (i2c_bus == nullptr) { return false; }
    i2c_master_dev_handle_t dev = ensure_i2c_master_dev(i2c_addr);
    if (!dev) { return false; }

    // Prepare buffer: register + payload on stack (no heap)
    static constexpr size_t kStackTxBuf = 64;
    if (size + 1 > kStackTxBuf) {
        DRV_LOGE("Fusb302HalEsp32: write_block too large ({} > {})", (unsigned)(size + 1), (unsigned)kStackTxBuf);
        return false;
    }
    uint8_t buf[kStackTxBuf];
    buf[0] = reg;
    if (size) { etl::copy(data, data + size, &buf[1]); }
    esp_err_t err = i2c_master_transmit(dev, buf, size + 1, I2C_TIMEOUT_MS);
    return err == ESP_OK;
#else
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
#endif
}

#if PD_ESP32_USE_NEW_I2C_MASTER_API
auto Fusb302RtosHalEsp32::ensure_i2c_master_dev(uint8_t addr) -> i2c_master_dev_handle_t {
    if (!i2c_bus) { return nullptr; }
    // Lock cache for lookup/create
    xSemaphoreTake(i2c_dev_cache_mutex, portMAX_DELAY);
    // Re-check after taking the lock
    auto it = i2c_dev_cache.find(addr);
    if (it != i2c_dev_cache.end()) {
        auto h = it->second;
        xSemaphoreGive(i2c_dev_cache_mutex);
        return h;
    }
    if (i2c_dev_cache.size() >= MAX_I2C_DEV_CACHE) {
        xSemaphoreGive(i2c_dev_cache_mutex);
        return nullptr; // logical capacity reached
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = i2c_freq;
    i2c_master_dev_handle_t dev = nullptr;
    esp_err_t add_res = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev);
    if (add_res != ESP_OK) {
        xSemaphoreGive(i2c_dev_cache_mutex);
        return nullptr;
    }
    i2c_dev_cache.insert(etl::pair<uint8_t, i2c_master_dev_handle_t>{addr, dev});
    xSemaphoreGive(i2c_dev_cache_mutex);
    return dev;
}
#endif

bool Fusb302RtosHalEsp32::read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t& data) {
    return read_block(i2c_addr, reg, &data, 1);
}

bool Fusb302RtosHalEsp32::write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) {
    return write_block(i2c_addr, reg, &data, 1);
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS_HAL_ESP32
