#pragma once

// Optionally, all settings can be defined in single external file,
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


#if !defined(PD_TIMER_RESOLUTION_US)
#define PD_TIMER_RESOLUTION_US 0
#endif
