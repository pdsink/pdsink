#pragma once

#include "logger.hpp"

#define PE_LOG(...) logger.push("pd.pe", jetlog::level::info, __VA_ARGS__)
#define PRL_LOG(...) logger.push("pd.prl", jetlog::level::info, __VA_ARGS__)
#define TC_LOG(...) logger.push("pd.tc", jetlog::level::info, __VA_ARGS__)
