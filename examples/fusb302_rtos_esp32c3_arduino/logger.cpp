//#include <Arduino.h>
#include "logger.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "hal/usb_serial_jtag_ll.h"
#include "rom/ets_sys.h"

static jetlog::RingBuffer<1024*10> ringBuffer;

Logger logger(ringBuffer);
jetlog::Reader<> logReader(ringBuffer);

/*
void logger_start() {
    xTaskCreate([](void* pvParameters) {
        (void)pvParameters;

        etl::string<1024> outputBuffer{};

        Serial.begin(115200);

        // Wait until serial connected, before printing. In other case
        // the log head from firmware start can be lost.
        while (!Serial) { vTaskDelay(pdMS_TO_TICKS(10)); }

        while (true) {
            // Read and print log records, until there is no more data.
            while (logReader.pull(outputBuffer)) {
                Serial.println(outputBuffer.c_str());
                outputBuffer.clear();
            }
            // If there is no data, wait a bit before trying again.
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "LogOutputTask", 1024 * 4, NULL, 0, NULL);
}
*/

// Use idf api for output. It's connection detector is more robust for re-uploads.
void logger_start() {
    xTaskCreate([](void* pvParameters) {
        (void)pvParameters;

        etl::string<1024> outputBuffer{};

        // Wait until usb serial ready, or startup messages will be lost
        while(!usb_serial_jtag_ll_txfifo_writable()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        while (true) {
            while (logReader.pull(outputBuffer)) {
                ets_printf("%s\n", outputBuffer.c_str());
                outputBuffer.clear();
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "LogOutputTask", 1024 * 4, NULL, 0, NULL);
}