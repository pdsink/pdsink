#pragma once

#include "logger.hpp"

#define PD_JETLOG_FN(...) logger.push(__VA_ARGS__)

// Use for debugging.
#define PD_LOG_DEFAULT ERROR

#define PD_LOG_PE INFO
// #define PD_LOG_PRL INFO
// #define PD_LOG_DRV INFO
// #define PD_LOG_TC INFO
// #define PD_LOG_DPM VERBOSE
#define PD_LOG_DPM INFO

#define USE_FUSB302_RTOS
#define USE_FUSB302_RTOS_HAL_ESP32
