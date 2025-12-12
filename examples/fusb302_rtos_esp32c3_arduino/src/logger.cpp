//#include <Arduino.h>
#include "logger.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "hal/usb_serial_jtag_ll.h"
#include "rom/ets_sys.h"

static jetlog::RingBuffer<1024*40> ringBuffer;

Logger logger(ringBuffer);
jetlog::Reader<> logReader(ringBuffer);

static etl::string<1024> outputBuffer{};
// Join multiple records into a batch. That significantly improves
// USB VCOM throughput.
static etl::string<4096> batchBuffer{};

// Use the IDF API for output; its connection detector is more robust for re-uploads.
void logger_start() {
    xTaskCreate([](void* pvParameters) {
        (void)pvParameters;

        // Required to properly track empty strings
        bool has_pending_str{false};

        static_assert(batchBuffer.MAX_SIZE > outputBuffer.MAX_SIZE + 1,
            "Batch buffer must be larger than output buffer");

        while (true) {
            // Wait until USB serial is ready before writing
            if (!usb_serial_jtag_ll_txfifo_writable()) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            while (logReader.pull(outputBuffer)) {
                has_pending_str = true;

                if (outputBuffer.size() + 1 <= batchBuffer.available()) {
                    batchBuffer += outputBuffer;
                    batchBuffer += '\n';
                    outputBuffer.clear();
                    has_pending_str = false;
                    continue;
                }

                // Force a batch flush when available space ends
                if (batchBuffer.size() > 0) {
                    printf("%s", batchBuffer.c_str());
                    batchBuffer.clear();

                    if (has_pending_str) {
                        batchBuffer += outputBuffer;
                        batchBuffer += '\n';
                        outputBuffer.clear();
                        has_pending_str = false;
                        continue;
                    }
                }
            }

            if (batchBuffer.size() > 0) {
                printf("%s", batchBuffer.c_str());
                batchBuffer.clear();
                // New log records could arrive until we send.
                // Skip delay for better responsiveness.
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }, "LogOutputTask", 1024 * 4, NULL, 1, NULL);
}
