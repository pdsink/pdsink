#pragma once

// Optionally, all settings can be defined in single external file,
// instead of build flags.
#if defined(__has_include)
  #if __has_include("pd_config.h")
    #include "pd_config.h"
  #endif
#endif

#if !defined(PE_LOG)
#define PE_LOG(...)
#endif

#if !defined(PRL_LOG)
#define PRL_LOG(...)
#endif

#if !defined(TC_LOG)
#define TC_LOG(...)
#endif

#if !defined(DRV_LOG)
#define DRV_LOG(...)
#endif

#if !defined(DRV_ERR)
#define DRV_ERR(...)
#endif

#if !defined(PD_TIMER_RESOLUTION_US)
#define PD_TIMER_RESOLUTION_US 0
#endif
