#include <queue_sizes.h>
#include <micro/utils/log.hpp>

#if LOG_ENABLED
static_assert(sizeof(micro::LogMessage) == micro_sizeof_LogMessage, "Size of 'debug::LogMessage' does not match required log item size!");
#endif
