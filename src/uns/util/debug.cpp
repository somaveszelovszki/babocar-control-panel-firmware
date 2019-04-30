#include <uns/util/debug.hpp>
#include <config/cfg_board.hpp>
#include <stdarg.h>
#include <string.h>
#include <uns/container/RingBuffer.hpp>
#include <uns/container/Vec.hpp>
#include <uns/util/numeric.hpp>
#include <uns/util/convert.hpp>
#include <uns/bsp/queue.hpp>

using namespace uns;

#define STR_LEN_FLOAT_DEC   (1 + 8)         // sign + decimal
#define STR_LEN_FLOAT_FRAC  4               // fraction
#define STR_FLOAT_FRAG_MUL  10000           // Multiplier of fragment: 10^4
#define STR_LEN_FLOAT       (1 + 8 + 1 + 4) // sign + decimal + '.' + fragment

#define MAX_TX_MSG_SIZE     1024u           // max size of transmit messages

uint32_t debug::Msg::append(const char* s, uint32_t len) {
    if (!len) {
        len = strlen(s);
    }

    return this->text.append(s, len);
}

uint32_t debug::Msg::append(char c) {
    return this->append(&c, 1);
}

#ifdef DEBUG
void debug::printf(uint32_t content, const char *format, va_list args, Status result) {
    Status status = Status::OK;
    Msg txMsg;
    txMsg.content = content;

    char numBuff[max(STR_MAX_LEN_INT, STR_LEN_FLOAT) + 1];

    uint32_t n = 0; // will store the index of the current character

    while (format[n] != '\0' && isOk(status)) {
        uint32_t prevSize = txMsg.text.size;
        if (format[n] != '%')
            txMsg.append(format[n]);
        else {
            switch (format[++n]) {
            case 's':
                txMsg.append(va_arg(args, char *));
                break;
            case 'c':
                txMsg.append(static_cast<char>(va_arg(args, int)));
                break;
            case 'd':
                if (uns::itoa(va_arg(args, int), numBuff, STR_MAX_LEN_INT) > 0) {
                    txMsg.append(numBuff);
                }
                break;
            case 'f':
                if (uns::ftoa(static_cast<float32_t>(va_arg(args, double)), numBuff, STR_LEN_FLOAT_DEC, STR_LEN_FLOAT_FRAC) > 0) {
                    txMsg.append(numBuff);
                }
                break;
            }
        }

        if (prevSize == txMsg.text.size) {
            status = Status::BUFFER_FULL;
        }

        ++n;
    }

    if (isOk(status)) {
        // appends result if needed (in case of error logs the result status will be appended)
        if (!isOk(result)) {
            if (!txMsg.append(" Result status: ") || !txMsg.append(uns::getStatusString(result))) {
                status = Status::BUFFER_FULL;
            }
        }

        if (isOk(status)) {
            uns::queueSend(cfg::queue_Log, &txMsg);
        }
    }
}

void debug::printf(uint32_t content, const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug::printf(content, format, args);
    va_end(args);
}

void debug::printlog(const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug::printf(CONTENT_FLAG_LOG, format, args);
    va_end(args);
}

void debug::printerr(Status result, const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug::printf(CONTENT_FLAG_ERROR, format, args, result);
    va_end(args);
}

void debug::printACK() {
    debug::printf(CONTENT_FLAG_ACK, "0");
}

#else
// Empty implementation for non-debug configuration.
void debug::printf(uint32_t content, const char * const format, va_list args, Status result) {
    (void)content;
    (void)format;
    (void)args;
    (void)result;
}

// Empty implementation for non-debug configuration.
void debug::printf(uint32_t content, const char * const format, ...) {
    (void)content;
    (void)format;
}

// Empty implementation for non-debug configuration.
void debug::printerr(Status result, const char * const format, ...) {
    (void)result;
    (void)format;
}

#endif // DEBUG
