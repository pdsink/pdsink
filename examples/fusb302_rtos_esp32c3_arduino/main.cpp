#include "pd.h"
#include "logger.hpp"
#include "blinker.hpp"

// For HW with disconnected VBUS pin. Let's see if that works.
class Driver : public pd::fusb302::Fusb302Rtos {
public:
    Driver(pd::Port& port, pd::Task& task, pd::fusb302::Fusb302RtosHalEsp32& hal)
        : Fusb302Rtos(port, task, hal) {}

    bool is_vbus_ok() override { return true; }
};


pd::Port port;
pd::Task task(port);
pd::DPM dpm(port, task);

pd::fusb302::Fusb302RtosHalEsp32 fusb302_hal;
Driver driver(port, task, fusb302_hal);

pd::PRL prl(port, task, driver);
pd::PE pe(port, task, dpm, prl, driver);
pd::TC tc(port, task, driver);

Blinker<LedDriver> blinker;


void setup() {
    logger_start();
    blinker.start();
    // LED animation example
    blinker.loop({ {{255,0,0}, 1000}, {{0,255,0}, 1000}, {{0,0,255}, 1000}, {{128,128,128}, 1000} });

    //
    // Devboard specific setup, not PD related
    //

    // FAN off.
    pinMode(21, OUTPUT); digitalWrite(21, LOW);
    // BUZZER off.
    pinMode(10, OUTPUT); digitalWrite(10, LOW);
    pinMode(8, OUTPUT); digitalWrite(8, LOW);
    // Heater off.
    pinMode(20, OUTPUT); digitalWrite(20, LOW);

    // Activate PD stack
    task.start();

    LOG_INFO("Setup complete");
}

void loop() {}
