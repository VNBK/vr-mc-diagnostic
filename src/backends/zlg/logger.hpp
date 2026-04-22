/*
 * logger.hpp — minimal printf-style LOG_* shim for the ZLG driver.
 *
 * The ZLG sources were imported from vr-hand-sdk where the shim forwards
 * to vrLogger. vr-mc-diagnostic has no vrLogger, so we write the
 * formatted line straight to stderr. Keeps the driver unchanged.
 */

#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>

namespace vrmc_zlg_log {

/* printf body → std::string via vsnprintf; ~512 chars of headroom. */
inline std::string vformat(const char* fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return std::string(buf);
}

inline void emit(const char* lvl, const char* tag, const std::string& msg)
{
    std::fprintf(stderr, "[%s] %s: %s\n", lvl, tag, msg.c_str());
}

}  // namespace vrmc_zlg_log

#ifdef LOG_TRACE
#  undef LOG_TRACE
#endif
#ifdef LOG_DEBUG
#  undef LOG_DEBUG
#endif
#ifdef LOG_INFO
#  undef LOG_INFO
#endif
#ifdef LOG_WARN
#  undef LOG_WARN
#endif
#ifdef LOG_ERROR
#  undef LOG_ERROR
#endif
#ifdef LOG_CRITICAL
#  undef LOG_CRITICAL
#endif

#define VRMC_ZLG_LOG(lvl, tag, ...) \
    vrmc_zlg_log::emit(lvl, (tag), vrmc_zlg_log::vformat(__VA_ARGS__))

#define LOG_TRACE(tag, ...)    VRMC_ZLG_LOG("TRACE",    tag, __VA_ARGS__)
#define LOG_DEBUG(tag, ...)    VRMC_ZLG_LOG("DEBUG",    tag, __VA_ARGS__)
#define LOG_INFO(tag, ...)     VRMC_ZLG_LOG("INFO",     tag, __VA_ARGS__)
#define LOG_WARN(tag, ...)     VRMC_ZLG_LOG("WARN",     tag, __VA_ARGS__)
#define LOG_ERROR(tag, ...)    VRMC_ZLG_LOG("ERROR",    tag, __VA_ARGS__)
#define LOG_CRITICAL(tag, ...) VRMC_ZLG_LOG("CRITICAL", tag, __VA_ARGS__)
