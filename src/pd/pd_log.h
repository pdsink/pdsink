#pragma once

#include "pd_conf.h"

#if defined(PD_LOG_FN) && defined(PD_JETLOG_FN)
#error "Define only one of PD_LOG_FN and PD_JETLOG_FN"
#endif

#if defined(PD_JETLOG_FN)
#include <jetlog/jetlog.hpp>

// Share one options object per enabled tag/level across calls and translation units.
#define _PD_DECLARE_LOG_OPTIONS(name, tag, lvl) \
    namespace jetlog_options { \
        inline constexpr jetlog::PushOptions name{jetlog::static_str(tag), jetlog::level::lvl}; \
    }
#define _PD_LOG(name, tag, lvl, ...) PD_JETLOG_FN(::jetlog_options::name, __VA_ARGS__)
#else
#define _PD_DECLARE_LOG_OPTIONS(name, tag, lvl)
#define _PD_LOG(name, tag, lvl, ...) PD_LOG_FN(tag, PD_LOG_FN_LVL_##lvl, __VA_ARGS__)
#endif

#ifndef PD_LOG_ASSUME_STATIC_STR
    #if defined(PD_JETLOG_FN)
        #define PD_LOG_ASSUME_STATIC_STR(s) jetlog::static_str((s))
    #else
        #define PD_LOG_ASSUME_STATIC_STR(s) (s)
    #endif
#endif

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
    _PD_DECLARE_LOG_OPTIONS(pe_error, "pd.pe", error)
    #define PE_LOGE(...) _PD_LOG(pe_error, "pd.pe", ERROR, __VA_ARGS__)
#else
  #define PE_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(pe_info, "pd.pe", info)
    #define PE_LOGI(...) _PD_LOG(pe_info, "pd.pe", INFO, __VA_ARGS__)
#else
    #define PE_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(pe_debug, "pd.pe", debug)
    #define PE_LOGD(...) _PD_LOG(pe_debug, "pd.pe", DEBUG, __VA_ARGS__)
#else
    #define PE_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PE) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(pe_verbose, "pd.pe", verbose)
    #define PE_LOGV(...) _PD_LOG(pe_verbose, "pd.pe", VERBOSE, __VA_ARGS__)
#else
    #define PE_LOGV(...) ((void)0)
#endif

//
// PRL
//
#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_ERROR
    _PD_DECLARE_LOG_OPTIONS(prl_error, "pd.prl", error)
    #define PRL_LOGE(...) _PD_LOG(prl_error, "pd.prl", ERROR, __VA_ARGS__)
#else
    #define PRL_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(prl_info, "pd.prl", info)
    #define PRL_LOGI(...) _PD_LOG(prl_info, "pd.prl", INFO, __VA_ARGS__)
#else
    #define PRL_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(prl_debug, "pd.prl", debug)
    #define PRL_LOGD(...) _PD_LOG(prl_debug, "pd.prl", DEBUG, __VA_ARGS__)
#else
    #define PRL_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_PRL) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(prl_verbose, "pd.prl", verbose)
    #define PRL_LOGV(...) _PD_LOG(prl_verbose, "pd.prl", VERBOSE, __VA_ARGS__)
#else
    #define PRL_LOGV(...) ((void)0)
#endif

//
// TC
//
#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_ERROR
    _PD_DECLARE_LOG_OPTIONS(tc_error, "pd.tc", error)
    #define TC_LOGE(...) _PD_LOG(tc_error, "pd.tc", ERROR, __VA_ARGS__)
#else
    #define TC_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(tc_info, "pd.tc", info)
    #define TC_LOGI(...) _PD_LOG(tc_info, "pd.tc", INFO, __VA_ARGS__)
#else
    #define TC_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(tc_debug, "pd.tc", debug)
    #define TC_LOGD(...) _PD_LOG(tc_debug, "pd.tc", DEBUG, __VA_ARGS__)
#else
    #define TC_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TC) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(tc_verbose, "pd.tc", verbose)
    #define TC_LOGV(...) _PD_LOG(tc_verbose, "pd.tc", VERBOSE, __VA_ARGS__)
#else
    #define TC_LOGV(...) ((void)0)
#endif

//
// DRV
//
#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_ERROR
    _PD_DECLARE_LOG_OPTIONS(drv_error, "pd.drv", error)
    #define DRV_LOGE(...) _PD_LOG(drv_error, "pd.drv", ERROR, __VA_ARGS__)
#else
    #define DRV_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(drv_info, "pd.drv", info)
    #define DRV_LOGI(...) _PD_LOG(drv_info, "pd.drv", INFO, __VA_ARGS__)
#else
    #define DRV_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(drv_debug, "pd.drv", debug)
    #define DRV_LOGD(...) _PD_LOG(drv_debug, "pd.drv", DEBUG, __VA_ARGS__)
#else
    #define DRV_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DRV) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(drv_verbose, "pd.drv", verbose)
    #define DRV_LOGV(...) _PD_LOG(drv_verbose, "pd.drv", VERBOSE, __VA_ARGS__)
#else
    #define DRV_LOGV(...) ((void)0)
#endif

//
// DPM
//
#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_ERROR
    _PD_DECLARE_LOG_OPTIONS(dpm_error, "pd.dpm", error)
    #define DPM_LOGE(...) _PD_LOG(dpm_error, "pd.dpm", ERROR, __VA_ARGS__)
#else
    #define DPM_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(dpm_info, "pd.dpm", info)
    #define DPM_LOGI(...) _PD_LOG(dpm_info, "pd.dpm", INFO, __VA_ARGS__)
#else
    #define DPM_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(dpm_debug, "pd.dpm", debug)
    #define DPM_LOGD(...) _PD_LOG(dpm_debug, "pd.dpm", DEBUG, __VA_ARGS__)
#else
    #define DPM_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_DPM) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(dpm_verbose, "pd.dpm", verbose)
    #define DPM_LOGV(...) _PD_LOG(dpm_verbose, "pd.dpm", VERBOSE, __VA_ARGS__)
#else
    #define DPM_LOGV(...) ((void)0)
#endif

//
// TASK
//
#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_ERROR
    _PD_DECLARE_LOG_OPTIONS(task_error, "pd.task", error)
    #define TASK_LOGE(...) _PD_LOG(task_error, "pd.task", ERROR, __VA_ARGS__)
#else
    #define TASK_LOGE(...)   ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_INFO
    _PD_DECLARE_LOG_OPTIONS(task_info, "pd.task", info)
    #define TASK_LOGI(...) _PD_LOG(task_info, "pd.task", INFO, __VA_ARGS__)
#else
    #define TASK_LOGI(...)  ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_DEBUG
    _PD_DECLARE_LOG_OPTIONS(task_debug, "pd.task", debug)
    #define TASK_LOGD(...) _PD_LOG(task_debug, "pd.task", DEBUG, __VA_ARGS__)
#else
    #define TASK_LOGD(...) ((void)0)
#endif

#if _PD_LOG_LVL_NUM(PD_LOG_TASK) >= _PD_LOG_LVL_VERBOSE
    _PD_DECLARE_LOG_OPTIONS(task_verbose, "pd.task", verbose)
    #define TASK_LOGV(...) _PD_LOG(task_verbose, "pd.task", VERBOSE, __VA_ARGS__)
#else
    #define TASK_LOGV(...) ((void)0)
#endif

#undef _PD_DECLARE_LOG_OPTIONS
