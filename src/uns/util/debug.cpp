#include <uns/util/debug.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/config/cfg_os.hpp>
#include <uns/config/cfg_perf.hpp>
#include <uns/container/vec.hpp>
#include <uns/container/RingBuffer.hpp>
#include <uns/util/numeric.hpp>
#include <uns/util/convert.hpp>
#include <uns/bsp/queue.hpp>

#include <cstdarg>
#include <cstring>

namespace uns {
namespace debug {

constexpr uint32_t STR_MAX_LEN_INT        = 1 + 10;         // sign + decimal
constexpr uint32_t STR_MAX_LEN_FLOAT_DEC  = 1 + 8;          // sign + decimal
constexpr uint32_t STR_MAX_LEN_FLOAT_FRAC = 4;              // fraction
constexpr uint32_t STR_MAX_LEN_FLOAT      = 1 + 8 + 1 + 4;  // sign + decimal + '.' + fragment

//void Log::append(const char* s, uint32_t len) {
//    if (!len) {
//        len = strlen(s);
//    }
//    this->text.append(s, s + len);
//}
//
//void Log::append(char c) {
//    this->append(&c, 1);
//}

#if LOGGING_ENABLED
void printlog(LogLevel level, const char *format, va_list args, Status status) {
    LogMessage msg;

    // TODO handle level
    char numBuf[uns::max(STR_MAX_LEN_INT, STR_MAX_LEN_FLOAT) + 1];

    uint32_t n = 0; // will store the index of the current character

    while (format[n] != '\0') {

        if (format[n] != '%')
            msg.append(format[n]);
        else {
            switch (format[++n]) {
            case 's':
            {
                const char *str = va_arg(args, char*);
                msg.append(str, str + strlen(str));
                break;
            }

            case 'c':
                msg.append(static_cast<char>(va_arg(args, int)));
                break;

            case 'd':
            {
                const uint32_t len = uns::itoa(va_arg(args, int), numBuf, STR_MAX_LEN_INT);
                if (len > 0) {
                    msg.append(numBuf, numBuf + strlen(numBuf));
                }
                break;
            }

            case 'f':
            {
                const uint32_t len = uns::ftoa(static_cast<float32_t>(va_arg(args, double)), numBuf, STR_MAX_LEN_FLOAT_DEC, STR_MAX_LEN_FLOAT_FRAC);
                if (len > 0) {
                    msg.append(numBuf, numBuf + strlen(numBuf));
                }
                break;
            }
            default:
                // Unsupported printf modifier
                break;
            }
        }

        ++n;
    }

    // appends result if needed (in case of error logs the result status will be appended)
    if (!isOk(status)) {
        static const char *statusTemplate = " | Status: ";
        static const uint32_t statusTemplateLen = strlen(statusTemplate);

        const char *statusStr = uns::getStatusString(status);

        msg.append(statusTemplate, statusTemplate + statusTemplateLen);
        msg.append(statusStr, statusStr + strlen(statusStr));
    }

    if (msg.append('\0')) {
        uns::queueSend(cfg::queue_Log, &msg);
    }
}

void printlog(LogLevel level, const char *format, ...) {
    va_list args;
    va_start(args, format);
    printlog(level, format, args);
    va_end(args);
}

void printerr(Status status, const char *format, ...) {
    va_list args;
    va_start(args, format);
    printlog(LogLevel::Error, format, args, status);
    va_end(args);
}

#else
// Empty implementation for non-debug configuration.
void printlog(LogLevel, const char*, va_list, Status) {}

// Empty implementation for non-debug configuration.
void printlog(LogLevel, const char*, ...) {}

// Empty implementation for non-debug configuration.
void printerr(Status, const char*, ...) {}

#endif // LOGGING_ENABLED

} // namespace debug
} // namesapce uns
