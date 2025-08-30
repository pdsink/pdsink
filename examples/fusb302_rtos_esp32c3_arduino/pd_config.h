#pragma once

#include "logger.hpp"

#define PD_LOG_FN(...)          logger.push(__VA_ARGS__)
#define PD_LOG_FN_LVL_ERR       jetlog::level::error
#define PD_LOG_FN_LVL_INFO      jetlog::level::info
#define PD_LOG_FN_LVL_DEBUG     jetlog::level::debug
#define PD_LOG_FN_LVL_VERBOSE   jetlog::level::verbose

// Use for debugging
// #define PD_LOG_DEFAULT DEBUG

// #define PD_LOG_PE INFO
// #define PD_LOG_PRL INFO
#define PD_LOG_TC INFO
#define PD_LOG_DPM INFO


#define USE_FUSB302_RTOS
#define USE_FUSB302_RTOS_HAL_ESP32
