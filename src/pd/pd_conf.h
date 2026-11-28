#pragma once

// Optionally, all settings can be defined in a single external file,
// instead of build flags.
#define PD_QUOTE_STR1(x)    #x
#define PD_QUOTE_STR(x)     PD_QUOTE_STR1(x)

#if defined(PD_CONFIG_FILE_STR)
    #include PD_CONFIG_FILE_STR
#elif defined(PD_CONFIG_FILE)
    #include PD_QUOTE_STR(PD_CONFIG_FILE)
#elif defined(PD_USE_CONFIG_FILE)
    #include "pd_config.h"
#endif

#ifndef PD_TIMER_TICK_US
#define PD_TIMER_TICK_US 50
#endif

#if PD_TIMER_TICK_US < 1 || PD_TIMER_TICK_US > 1000
#error "PD_TIMER_TICK_US must be between 1 and 1000 microseconds"
#endif

// Round up and add one tick to avoid expiring before the requested delay.
#define PD_US_TO_TICKS(us) \
    ((static_cast<unsigned long long>(us) + PD_TIMER_TICK_US - 1ULL) / \
     PD_TIMER_TICK_US + 1ULL)
