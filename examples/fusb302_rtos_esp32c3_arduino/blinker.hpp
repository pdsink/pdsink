#pragma once

//
// PWM / Timers usage:
//
// PWM0 / T0: LED (demo board only, real device uses RMT)
// PWM1 / T1: Fan speed
// PWM2+PWM3 / T2: Buzzer
//

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "blinker_engine.hpp"

#include <rmt_led_strip.hpp>

class LedDriver : public IBlinkerLED<3> {
private:
    static constexpr gpio_num_t ledPin{GPIO_NUM_0};

    htcw::ws2812 led{ledPin, 1};
    DataType buffer{};

public:
    using DataType = typename IBlinkerLED<3>::DataType;

    LedDriver() {
        led.initialize();
    }

    void set(const DataType &val) override {
        if (val == buffer) return;
        buffer = val; // persist for async transfer

        led.color(0, buffer[0], buffer[1], buffer[2]);
        led.update();
    }
};


template <typename Driver>
class Blinker : public BlinkerEngine<Driver> {
private:
    TimerHandle_t timer;

    static void timerCallback(TimerHandle_t timer) {
        Blinker* self = static_cast<Blinker*>(pvTimerGetTimerID(timer));
        self->tick(esp_timer_get_time() / 1000);
    }

public:
    void start() {
        timer = xTimerCreate(
            "BlinkerTimer",
            pdMS_TO_TICKS(20),
            pdTRUE,
            this,
            timerCallback
        );
        xTimerStart(timer, 0);
    }
};