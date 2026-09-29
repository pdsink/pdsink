#include "../pd_conf.h"
#include "../pd_log.h"
#include <etl/algorithm.h>

#if defined(USE_FUSB302_RTOS_HAL_ESP32)

#include "fusb302_rtos_hal_esp32.h"

#include "esp_idf_version.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
#error "ESP‑IDF ≥ 4.4.0 required"
#endif

#if !defined(USE_FUSB302_COARSE_TIMER)
#include <soc/soc_caps.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/gptimer.h>
#include <hal/timer_ll.h>
#else
#include <driver/timer.h>
#endif
#endif

namespace pd {

namespace fusb302 {

Fusb302RtosHalEsp32::Fusb302RtosHalEsp32() {
}

#if defined(USE_FUSB302_COARSE_TIMER)
static_assert(configTICK_RATE_HZ >= 1000,
              "Coarse FUSB302 timer requires a FreeRTOS tick rate of at least 1000Hz");

static uint32_t rtos_ticks_to_us(TickType_t ticks) {
    return static_cast<uint32_t>(
        static_cast<uint64_t>(ticks) * 1000000ULL / configTICK_RATE_HZ
    );
}
#endif

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
        // Ignore the error if someone already called i2c_driver_install;
        // this is an expected use case. Report all other errors.
        ESP_ERROR_CHECK(err);
    }

    // Enable i2c SDA/SCL lines filter with defaults to suppress spikes
    ESP_ERROR_CHECK(i2c_filter_enable(i2c_num, 7));
}

void Fusb302RtosHalEsp32::init_timer() {
#if defined(USE_FUSB302_COARSE_TIMER)
    timer_handle = xTimerCreate(
        "Fusb302Timer",
        1,
        pdFALSE,
        this,
        [](TimerHandle_t timer) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(pvTimerGetTimerID(timer));
            if (self->event_cb) {
                self->event_cb(HAL_EVENT_TYPE::Timer, false);
            }
        }
    );
    configASSERT(timer_handle != nullptr);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
        .flags = {},
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &timer_handle));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = [](gptimer_handle_t, const gptimer_alarm_event_data_t*, void* arg) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
            if (self->event_cb) {
                self->event_cb(HAL_EVENT_TYPE::Timer, true);
            }
            return false;
        },
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &callbacks, this));
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
#else
    // Legacy support for PlatformIO's Arduino build only. Keep timer selection
    // fixed instead of adding legacy-specific options to the public HAL API.
    // For custom high-resolution timing on Arduino, override the HAL timer
    // methods together; otherwise define USE_FUSB302_COARSE_TIMER.
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider = TIMER_BASE_CLK / 1000000,
#if SOC_TIMER_GROUP_SUPPORT_XTAL
        .clk_src = TIMER_SRC_CLK_APB,
#endif
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));
    ESP_ERROR_CHECK(timer_isr_callback_add(
        TIMER_GROUP_0,
        TIMER_0,
        [](void* arg) {
            auto* self = static_cast<Fusb302RtosHalEsp32*>(arg);
            if (self->event_cb) {
                self->event_cb(HAL_EVENT_TYPE::Timer, true);
            }
            return false;
        },
        this,
        0
    ));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
#endif
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
        // Ignore the error if someone already called gpio_install_isr_service;
        // this is an expected use case. Report all other errors.
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
#if defined(USE_FUSB302_COARSE_TIMER)
        xTimerDelete(timer_handle, portMAX_DELAY);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ESP_ERROR_CHECK(gptimer_stop(timer_handle));
        ESP_ERROR_CHECK(gptimer_disable(timer_handle));
        ESP_ERROR_CHECK(gptimer_del_timer(timer_handle));
#else
        ESP_ERROR_CHECK(timer_pause(TIMER_GROUP_0, TIMER_0));
        ESP_ERROR_CHECK(timer_isr_callback_remove(TIMER_GROUP_0, TIMER_0));
        ESP_ERROR_CHECK(timer_deinit(TIMER_GROUP_0, TIMER_0));
#endif
        gpio_isr_handler_remove(int_io_pin);

        i2c_driver_delete(i2c_num);
    }
}

auto Fusb302RtosHalEsp32::get_tick_source() const -> ITimer::TickSource {
    return ITimer::TickSource::create<Fusb302RtosHalEsp32, &Fusb302RtosHalEsp32::get_ticks>(*this);
}

uint32_t Fusb302RtosHalEsp32::get_ticks() const {
#if defined(USE_FUSB302_COARSE_TIMER)
    TickType_t ticks = xPortInIsrContext() ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
    return rtos_ticks_to_us(ticks);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (!started) { return 0; }

    uint64_t raw = 0;
    ESP_ERROR_CHECK(gptimer_get_raw_count(timer_handle, &raw));
    return static_cast<uint32_t>(raw);
#else
    if (!started) { return 0; }

    uint64_t raw = 0;
    ESP_ERROR_CHECK(timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &raw));
    return static_cast<uint32_t>(raw);
#endif
}

void Fusb302RtosHalEsp32::rearm(uint32_t deadline_ticks) {
#if defined(USE_FUSB302_COARSE_TIMER)
    TickType_t anchor_ticks = xTaskGetTickCount();
    int32_t delta = static_cast<int32_t>(
        deadline_ticks - rtos_ticks_to_us(anchor_ticks)
    );
    TickType_t delay_ticks = 1;
    if (delta > 0) {
        delay_ticks = static_cast<TickType_t>(
            (static_cast<uint64_t>(delta) * configTICK_RATE_HZ + 999999ULL) / 1000000ULL
        );
        if (delay_ticks == 0) { delay_ticks = 1; }
    }

    BaseType_t result = xTimerChangePeriod(timer_handle, delay_ticks, portMAX_DELAY);
    configASSERT(result == pdPASS);
    if (result == pdPASS) {
        // CHANGE_PERIOD is anchored when the daemon processes its queue.
        // RESET carries our original sample, restoring the absolute deadline.
        result = xTimerGenericCommand(
            timer_handle,
            tmrCOMMAND_RESET,
            anchor_ticks,
            nullptr,
            portMAX_DELAY
        );
        configASSERT(result == pdPASS);
    }
    (void)result;
#else
    configASSERT(started);

    uint64_t raw = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(gptimer_get_raw_count(timer_handle, &raw));
#else
    ESP_ERROR_CHECK(timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &raw));
#endif
#if defined(TIMER_LL_COUNTER_BIT_WIDTH)
    constexpr uint64_t counter_mask = UINT64_MAX >> (64 - TIMER_LL_COUNTER_BIT_WIDTH);
#else
    constexpr uint64_t counter_mask = UINT64_MAX >> (64 - SOC_TIMER_GROUP_COUNTER_BIT_WIDTH);
#endif
    int32_t delta = static_cast<int32_t>(deadline_ticks - static_cast<uint32_t>(raw));
    uint64_t alarm = (raw + static_cast<int64_t>(delta)) & counter_mask;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = alarm,
        .reload_count = 0,
        .flags = {},
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));
#else
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarm));
    ESP_ERROR_CHECK(timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN));
#endif
#endif

    if (static_cast<int32_t>(deadline_ticks - get_ticks()) <= 0 && event_cb) {
        event_cb(HAL_EVENT_TYPE::Timer, false);
    }
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
    if (!i2c_initialized) { return false; }
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
