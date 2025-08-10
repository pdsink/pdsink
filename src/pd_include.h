#pragma once

//
// Generic headers.
//

#include "dpm.h"
#include "idriver.h"
#include "pe.h"
#include "port.h"
#include "prl.h"
#include "task.h"
#include "tc.h"

//
// Built-in drivers, if enabled by user.
//

#ifdef USE_FUSB302_RTOS
#include "drivers/fusb302_rtos.h"
#endif // USE_FUSB302_RTOS

#ifdef USE_FUSB302_RTOS_HAL_ESP32
#include "drivers/fusb302_rtos_hal_esp32.h"
#endif // USE_FUSB302_RTOS_HAL_ESP32
