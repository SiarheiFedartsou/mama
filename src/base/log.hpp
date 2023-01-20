#pragma once

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#define MAMA_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define MAMA_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define MAMA_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define MAMA_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define MAMA_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define MAMA_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

namespace mama {
namespace base {
inline void InitializeLogging() { spdlog::cfg::load_env_levels(); }
} // namespace base
} // namespace mama