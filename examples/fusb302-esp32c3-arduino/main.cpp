#include "pd.h"
#include "logger.hpp"
#include "blinker.hpp"

//pd::Fusb302Esp32Arduino fusb302;
pd::Sink sink;
pd::DummyDriver driver(sink);
pd::PRL prl(sink, driver);
pd::PE pe(sink, prl, driver);
pd::TC tc(sink, driver);
pd::DPM dpm(sink, pe);

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

    LOG_INFO("Setup complete");
}

void loop() {}
