#pragma once

#include "logger.hpp"

#define PE_LOG(...) logger.push("pd.pe", jetlog::level::info, __VA_ARGS__)
#define PRL_LOG(...) logger.push("pd.prl", jetlog::level::info, __VA_ARGS__)
#define TC_LOG(...) logger.push("pd.tc", jetlog::level::info, __VA_ARGS__)
#define DRV_LOG(...) logger.push("pd.drv", jetlog::level::info, __VA_ARGS__)
#define DRV_ERR(...) logger.push("pd.drv", jetlog::level::error, __VA_ARGS__)

#define USE_FUSB302_RTOS
#define USE_FUSB302_RTOS_HAL_ESP32
