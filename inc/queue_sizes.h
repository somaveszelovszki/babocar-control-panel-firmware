#ifndef CAR_STM_QUEUE_SIZES_H
#define CAR_STM_QUEUE_SIZES_H

#include <micro/utils/types.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Size declarations for the OS queues.

#define LOG_MSG_MAX_SIZE    128     // Maximum size of log messages including message start character ('#'), debug code, separator (':') and message end characters ('\r\n').

#define micro_sizeof_LogMessage   (1 + 3 + 4 + LOG_MSG_MAX_SIZE) // Size of a debug::Log object.
#define micro_sizeof_ControlProps (4 + (4 + 4))              // Size of a ControlProps queue item (speed + sizeof(LineData)).

typedef uint8_t LogQueueItem_t[micro_sizeof_LogMessage];              // Defines buffer type of Log queue item.
typedef uint8_t ControlPropsQueueItem_t[micro_sizeof_ControlProps];   // Defines buffer type of ControlProps queue item.

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CAR_STM_QUEUE_SIZES_H
