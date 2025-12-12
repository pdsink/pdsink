#include <etl/array.h>
#include <etl/cyclic_value.h>
#include <etl/error_handler.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pd/pd.h>

#include "app_dpm.h"
#include "logger.hpp"
#include "blinker.hpp"

class Driver : public pd::fusb302::Fusb302Rtos {
public:
    Driver(pd::Port& port, pd::fusb302::Fusb302RtosHalEsp32& hal) : Fusb302Rtos(port, hal) {
        // Example of overriding default settings. You can do the same
        // for the HAL class to reassign IO pins and so on.
        task_stack_size_bytes = 1024 * 6;
        task_priority = 7;
    }
};

pd::Port port;
pd::fusb302::Fusb302RtosHalEsp32 fusb302_hal;
Driver driver(port, fusb302_hal);

pd::Task task(port, driver);
AppDPM dpm(port);
pd::PRL prl(port, driver);
pd::PE pe(port, dpm, prl, driver);
pd::TC tc(port, driver);

Blinker<LedDriver> blinker;

void etl_error_log(const etl::exception& e) {
    APP_LOGE("ETL Error: {}, file: {}, line: {}",
        e.what(), e.file_name(), e.line_number());
}

void initialize() {
    logger_start();
    blinker.start();

    // LED animation example
    blinker.loop({ {{255,0,0}, 1000}, {{0,255,0}, 1000}, {{0,0,255}, 1000}, {{128,128,128}, 1000} });

    // Activate PD stack
    task.start(tc, dpm, pe, prl, driver);
    // Preset desired voltage
    dpm.trigger_any(12000);

    // Simple task to cycle trigger_any every 5 seconds (ETL array + cyclic counter)
    xTaskCreate(
        [](void* /*pv*/){
            const etl::array mv_values{ 9000, 12000, 17000/*PPS*/ };
            etl::cyclic_value<uint8_t, 0, mv_values.SIZE - 1> idx{0};

            while (true) {
                vTaskDelay(pdMS_TO_TICKS(10*1000));
                const uint32_t mv = mv_values[idx];
                APP_LOGI("Trigger to {} mV", mv);
                dpm.trigger_any(mv);
                ++idx;
            }
        },
        "TriggerCycler", 1024 * 2, nullptr, 1, nullptr
    );

    // Register ETL error callback only when ETL error logging is enabled
#ifdef ETL_LOG_ERRORS
    etl::error_handler::set_callback<etl_error_log>();
#endif

    APP_LOGI("Setup complete");

}

#ifdef ARDUINO
void setup() { initialize(); }
void loop() {}
#else
extern "C" void app_main(void) {
    initialize();
    // Main task no longer needed once initialization completes.
    vTaskDelete(nullptr);
}
#endif
