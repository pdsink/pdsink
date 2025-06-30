#pragma once

#if __cplusplus >= 201703L
    #define __maybe_unused [[maybe_unused]]
#elif defined(__GNUC__) || defined(__clang__)
    #define __maybe_unused __attribute__((__unused__))
#elif defined(_MSC_VER)
    #define __maybe_unused __declspec(unused)
#else
    #define __maybe_unused
#endif


#if defined(__cplusplus) && __cplusplus >= 201703L
    #define __fallthrough [[fallthrough]]
#elif defined(__has_cpp_attribute)
    #if __has_cpp_attribute(fallthrough)
        #define __fallthrough [[fallthrough]]
    #endif
#endif
#ifndef __fallthrough
    #if defined(__GNUC__) || defined(__clang__)
        #define __fallthrough __attribute__((fallthrough))
    #else
        #define __fallthrough
    #endif
#endif
