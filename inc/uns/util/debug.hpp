#pragma once

#include <stdarg.h>
#include <uns/util/types.hpp>
#include <uns/bsp/uart.hpp>
#include <uns/container/Vec.hpp>

namespace uns {
namespace debug {

constexpr char MSG_START        = '#';      // Debug message start character.
constexpr char MSG_SEP          = ':';      // Debug message content flag and data separator character.
constexpr char MSG_END          = '\n';     // Debug message end character.
constexpr char MSG_VALUE_SEP    = '|';      // Debug message value separator.

constexpr uint32_t CONTENT_FLAG_SET_CONTENT   = 0x00000001;   ///< Debug content flag for setting content flags (app-to-controller).
constexpr uint32_t CONTENT_FLAG_CAR_PROPS     = 0x00000002;   ///< Debug content flag for car properties (controller-to-app).
constexpr uint32_t CONTENT_FLAG_OPTO_BITS     = 0x00000004;   ///< Debug content flag for optical sensor bits (controller-to-app).
constexpr uint32_t CONTENT_FLAG_LOG           = 0x00000008;   ///< Debug content flag for log messages (controller-to-app).
constexpr uint32_t CONTENT_FLAG_CONTROLLER    = 0x00000010;   ///< Debug content flag for controller messages (app-to-controller and controller-to-app).
constexpr uint32_t CONTENT_FLAG_GRAPH         = 0x00000020;   ///< Debug content flag for graph messages (controller-to-app).
constexpr uint32_t CONTENT_FLAG_ERROR         = 0x00000040;   ///< Debug content flag for error messages (controller-to-app).
constexpr uint32_t CONTENT_FLAG_ACK           = 0x00000080;   ///< Debug content flag for acknowledgements (app-to-controller and controller-to-app).
constexpr uint32_t CONTENT_FLAG_LINES         = 0x00000100;   ///< Debug content flag for line data (controller-to-app).
constexpr uint32_t CONTENT_FLAG_SPEED         = 0x00000200;   ///< Debug content flag for speed (app-to-controller and controller-to-app).

/* @brief Structure for storing debug messages.
 */
struct Msg {
    uint32_t content;               // The message content. @see debug::CONTENT_* variables.
    Vec<char, LOG_MSG_MAX_SIZE> text;   // The message text.

    /* @brief Appends string to the end of the message text.
     * @param s The string to append.
     * @param len The length of the string to append. If 0 is given, it will be determined run-time - 0 by default.
     * @returns Number of characters appended.
     */
    uint32_t append(const char *s, uint32_t len = 0);

    /* @brief Appends character to the end of the message text.
     * @param c The character to append.
     * @returns Number of characters appended.
     */
    uint32_t append(char c);
};

static_assert(sizeof(Msg) == uns_sizeof_DebugMsg, "Size of 'debug::Msg' does not match required log item size!");

/* @brief Prints a debug code and a string to the console through USART.
 * Supported modifiers : %s, %c, %d, %f
 * @param content The message content. @see uns::debug::CONTENT_* constants.
 * @param format The string format.
 * @param args Additional parameters.
 * @param result The result status. If not OK, it will be appended at the end of the log (e.g. '... Result status: BUSY').
 **/
void printf(uint32_t content, const char *format, va_list args, Status result = Status::OK);

/* @brief Prints a debug code and a string to the console through USART.
 * Supported modifiers : %s, %c, %d, %f
 * @param content The message content. @see uns::debug::CONTENT_* constants.
 * @param format The string format.
 * @params Additional parameters.
 **/
void printf(uint32_t content, const char *format, ...);

void printlog(const char *format, ...);

void printerr(Status result, const char *format, ...);

void printACK();

} // namespace debug
} // namespace uns


