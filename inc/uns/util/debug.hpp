#pragma once

#include <uns/config/cfg_perf.hpp>
#include <uns/container/vec.hpp>
#include <uns/util/types.hpp>
#include <uns/bsp/uart.hpp>

#include <cstdarg>

namespace uns {
namespace debug {

/* @brief Structure for storing debug messages.
 */
//struct Log {
//    LogLevel level;                     // The log message content.
//    vec<char, LOG_MSG_MAX_SIZE> text;   // The log message text.
//
//    /* @brief Appends string to the end of the message data.
//     * @param s The string to append.
//     * @param len The length of the string to append. If 0 is given, it will be determined run-time - 0 by default.
//     */
//    void append(const char *s, uint32_t len = 0);
//
//    /* @brief Appends character to the end of the message data.
//     * @param c The character to append.
//     */
//    void append(char c);
//};

typedef vec<char, LOG_MSG_MAX_SIZE> LogMessage;

static_assert(sizeof(LogMessage) == uns_sizeof_LogMessage, "Size of 'debug::LogMessage' does not match required log item size!");

/* @brief Prints a debug code and a string to the console through USART.
 * Supported modifiers : %s, %c, %d, %f
 * @param level The log level.
 * @param format The string format.
 * @param args Additional parameters.
 * @param status The status - if not OK, it will be appended at the end of the log (e.g. '... | Status: BUSY').
 **/
void printlog(LogLevel level, const char *format, va_list args, Status status = Status::OK);

/* @brief Prints a debug code and a string to the console through USART.
 * Supported modifiers : %s, %c, %d, %f
 * @param level The log level.
 * @param format The string format.
 * @params Additional parameters.
 **/
void printlog(LogLevel level, const char *format, ...);

void printerr(Status status, const char *format, ...);

} // namespace debug
} // namespace uns

#if LOGGING_ENABLED && 0

#define LOG_DEBUG(format, ...)      uns::debug::printlog(uns::LogLevel::Debug, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)       uns::debug::printlog(uns::LogLevel::Info, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...)    uns::debug::printlog(uns::LogLevel::Warning, format, ##__VA_ARGS__)
#define LOG_ERROR_WITH_STATUS(status, format, ...)  uns::debug::printerr(status, format, ##__VA_ARGS__)

#else

#define LOG_DEBUG(...)
#define LOG_INFO(...)
#define LOG_WARNING(...)
#define LOG_ERROR_WITH_STATUS(...)

#endif // LOGGING_ENABLED
