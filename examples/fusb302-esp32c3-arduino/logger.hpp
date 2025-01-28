#pragma once

#include "jetlog/jetlog.hpp"

//
// Here we create simple jetlog wrappers to use in project.
//

using Logger = jetlog::Writer<>;

extern Logger logger;

// We do not use tags in this example, so we pass empty string.
#define LOG_INFO(...) logger.push("DEMO_APP", jetlog::level::info, __VA_ARGS__)

void logger_start();
