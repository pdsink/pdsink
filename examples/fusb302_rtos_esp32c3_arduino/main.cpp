#include "pd.h"
#include "logger.hpp"
#include "blinker.hpp"

// For HW with disconnected VBUS pin, should work for sink with "dead battery".
// This may be useful for EPR mode to keep HW simple, because fusb302b allows
// 28v max at VBUS pin, and can die.
//
// But it's better not use this hack, and connect VBUS pin using resistor
// and TVS instead. Without EPR direct VBUS connection will be fine.
class Driver : public pd::fusb302::Fusb302Rtos {
public:
    Driver(pd::Port& port, pd::fusb302::Fusb302RtosHalEsp32& hal)
        : Fusb302Rtos(port, hal) {}

    bool is_vbus_ok() override { return true; }
};


pd::Port port;
pd::fusb302::Fusb302RtosHalEsp32 fusb302_hal;
Driver driver(port, fusb302_hal);

pd::Task task(port, driver);
pd::DPM dpm(port);
pd::PRL prl(port, driver);
pd::PE pe(port, dpm, prl, driver);
pd::TC tc(port, driver);

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
    task.start(tc, dpm, pe, prl, driver);

    LOG_INFO("Setup complete");
}

void loop() {}
