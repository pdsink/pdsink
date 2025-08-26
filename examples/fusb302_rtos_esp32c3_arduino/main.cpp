#include <etl/error_handler.h>
#include <pd/pd.h>

#include "app_dpm.h"
#include "logger.hpp"
#include "blinker.hpp"

class Driver : public pd::fusb302::Fusb302Rtos {
public:
    Driver(pd::Port& port, pd::fusb302::Fusb302RtosHalEsp32& hal) : Fusb302Rtos(port, hal) {
        // Example of default settings override. You can do the same
        // for HAL class to reassign IO pins and so on.
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

void setup() {
    logger_start();
    blinker.start();
    // LED animation example
    blinker.loop({ {{255,0,0}, 1000}, {{0,255,0}, 1000}, {{0,0,255}, 1000}, {{128,128,128}, 1000} });

    // Activate PD stack
    task.start(tc, dpm, pe, prl, driver);
    // Preset desired voltage
    dpm.trigger_fixed(12000);

    etl::error_handler::set_callback<etl_error_log>();

    struct test_exception : etl::exception {
        test_exception(string_type file, numeric_type line)
            : exception("Test exception", file, line) {}
    };
    ETL_ASSERT_FAIL(ETL_ERROR(test_exception)); // Test ETL error handling

    APP_LOGI("Setup complete");
}

void loop() {}
