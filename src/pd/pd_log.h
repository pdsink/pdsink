#pragma once

#if defined(PD_USE_JETLOG)
#include <jetlog/jetlog.hpp>
#endif

#include "pd_conf.h"

#define _PD_LOG_LVL_NONE   0
#define _PD_LOG_LVL_ERROR  1
#define _PD_LOG_LVL_INFO   2
#define _PD_LOG_LVL_DEBUG  3
#define _PD_LOG_LVL_VERBOSE  4

#define _PD_LOG_LVL_NUM(x) _PD_LOG_LVL_NUM_I(x)
#define _PD_LOG_LVL_NUM_I(x) _PD_LOG_LVL_##x

#ifndef PD_LOG_DEFAULT
#define PD_LOG_DEFAULT NONE
#endif

#ifndef PD_LOG_PE
#define PD_LOG_PE PD_LOG_DEFAULT
#endif

#ifndef PD_LOG_PRL
#define PD_LOG_PRL PD_LOG_DEFAULT
#endif

#ifndef PD_LOG_TC
#define PD_LOG_TC PD_LOG_DEFAULT
#endif

#ifndef PD_LOG_DRV
#define PD_LOG_DRV PD_LOG_DEFAULT
#endif

#ifndef PD_LOG_DPM
#define PD_LOG_DPM PD_LOG_DEFAULT
#endif

#ifndef PD_LOG_TASK
#define PD_LOG_TASK PD_LOG_DEFAULT
#endif

#define _PD_LOG_LVL_NONE_CHECK 1
#define _PD_LOG_LVL_ERROR_CHECK 1
#define _PD_LOG_LVL_INFO_CHECK 1
#define _PD_LOG_LVL_DEBUG_CHECK 1
#define _PD_LOG_LVL_VERBOSE_CHECK 1

#define _PD_CHECK_VALID(x) _PD_CHECK_VALID_I(x)
#define _PD_CHECK_VALID_I(x) _PD_LOG_LVL_##x##_CHECK

static_assert(_PD_CHECK_VALID(PD_LOG_PE),
    "PD_LOG_PE must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");
static_assert(_PD_CHECK_VALID(PD_LOG_PRL),
    "PD_LOG_PRL must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");
static_assert(_PD_CHECK_VALID(PD_LOG_TC),
    "PD_LOG_TC must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");
static_assert(_PD_CHECK_VALID(PD_LOG_DRV),
    "PD_LOG_DRV must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");
static_assert(_PD_CHECK_VALID(PD_LOG_DPM),
    "PD_LOG_DPM must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");
static_assert(_PD_CHECK_VALID(PD_LOG_TASK),
    "PD_LOG_TASK must be one of: NONE, ERROR, INFO, DEBUG, VERBOSE");

#undef _PD_CHECK_VALID
#undef _PD_CHECK_VALID_I

//
// PE
//
#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_ERROR
  #define PE_LOGE(...)   PD_LOG_FN("pd.pe", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
  #define PE_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_INFO
    #define PE_LOGI(...)  PD_LOG_FN("pd.pe", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define PE_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_DEBUG
    #define PE_LOGD(...) PD_LOG_FN("pd.pe", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define PE_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_VERBOSE
    #define PE_LOGV(...) PD_LOG_FN("pd.pe", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define PE_LOGV(...) ((void)0)
#endif

//
// PRL
//
#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_ERROR
    #define PRL_LOGE(...)   PD_LOG_FN("pd.prl", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
    #define PRL_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_INFO
    #define PRL_LOGI(...)  PD_LOG_FN("pd.prl", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define PRL_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_DEBUG
    #define PRL_LOGD(...) PD_LOG_FN("pd.prl", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define PRL_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_VERBOSE
    #define PRL_LOGV(...) PD_LOG_FN("pd.prl", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define PRL_LOGV(...) ((void)0)
#endif

//
// TC
//
#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_ERROR
    #define TC_LOGE(...)   PD_LOG_FN("pd.tc", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
    #define TC_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_INFO
    #define TC_LOGI(...)  PD_LOG_FN("pd.tc", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define TC_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_DEBUG
    #define TC_LOGD(...) PD_LOG_FN("pd.tc", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define TC_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_VERBOSE
    #define TC_LOGV(...) PD_LOG_FN("pd.tc", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define TC_LOGV(...) ((void)0)
#endif

//
// DRV
//
#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_ERROR
    #define DRV_LOGE(...)   PD_LOG_FN("pd.drv", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
    #define DRV_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_INFO
    #define DRV_LOGI(...)  PD_LOG_FN("pd.drv", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define DRV_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_DEBUG
    #define DRV_LOGD(...) PD_LOG_FN("pd.drv", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define DRV_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_VERBOSE
    #define DRV_LOGV(...) PD_LOG_FN("pd.drv", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define DRV_LOGV(...) ((void)0)
#endif

//
// DPM
//
#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_ERROR
    #define DPM_LOGE(...)   PD_LOG_FN("pd.dpm", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
    #define DPM_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_INFO
    #define DPM_LOGI(...)  PD_LOG_FN("pd.dpm", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define DPM_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_DEBUG
    #define DPM_LOGD(...) PD_LOG_FN("pd.dpm", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define DPM_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_VERBOSE
    #define DPM_LOGV(...) PD_LOG_FN("pd.dpm", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define DPM_LOGV(...) ((void)0)
#endif

//
// TASK
//
#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_ERROR
    #define TASK_LOGE(...)   PD_LOG_FN("pd.task", PD_LOG_FN_LVL_ERROR, __VA_ARGS__)
#else
    #define TASK_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_INFO
    #define TASK_LOGI(...)  PD_LOG_FN("pd.task", PD_LOG_FN_LVL_INFO, __VA_ARGS__)
#else
    #define TASK_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_DEBUG
    #define TASK_LOGD(...) PD_LOG_FN("pd.task", PD_LOG_FN_LVL_DEBUG, __VA_ARGS__)
#else
    #define TASK_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_VERBOSE
    #define TASK_LOGV(...) PD_LOG_FN("pd.task", PD_LOG_FN_LVL_VERBOSE, __VA_ARGS__)
#else
    #define TASK_LOGV(...) ((void)0)
#endif
