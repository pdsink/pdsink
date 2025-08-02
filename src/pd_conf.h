#pragma once

// Optionally, all settings can be defined in single external file,
// instead of build flags.
#if defined(__has_include)
  #if __has_include("pd_config.h")
    #include "pd_config.h"
  #endif
#endif

#if !defined(PD_TIMER_RESOLUTION_US)
#define PD_TIMER_RESOLUTION_US 0
#endif

#if !defined(PD_TIME_T)
#define PD_TIME_T uint32_t
#endif
