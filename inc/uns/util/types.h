#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef float float32_t;    // 32-bit floating point number
typedef double float64_t;   // 64-bit floating point number

typedef uint8_t uint1_t, uint2_t, uint3_t, uint4_t, uint5_t, uint6_t, uint7_t;   // types for storing 1, 2, 3, 4, 5, 6 and 7-bit values

typedef uint32_t res_id_t;  // Resource id type - used when referencing hardware modules.

// Size declarations - needed for the OS queues.

#define LOG_MSG_MAX_SIZE    128     // Maximum size of log messages including message start character ('#'), debug code, separator (':') and message end characters ('\r\n').
#define MAX_TX_BUFFER_SIZE  512u    // size of the log TX buffer
#define MAX_RX_BUFFER_SIZE  512u    // size of the log RX buffer

#define STR_MAX_LEN_GLOBAL_NAME 32u     // Maximum length of global parameter name.
#define STR_MAX_LEN_GLOBAL_TYPE 32u     // Maximum length of global parameter name.
#define MAX_NUM_GLOBAL_PARAMS   128u    // Maximum number of global parameters, that can be set from monitoring app.

#define uns_sizeof_LogMessage   (LOG_MSG_MAX_SIZE + 4)  // Size of a debug::Log object.
#define uns_sizeof_ControlProps (4 + (4 + 4))           // Size of a ControlProps queue item (speed + sizeof(LineData)).

typedef uint8_t LogQueueItem_t[uns_sizeof_LogMessage];              // Defines buffer type of Log queue item.
typedef uint8_t ControlPropsQueueItem_t[uns_sizeof_ControlProps];   // Defines buffer type of ControlProps queue item.

void onHardFault();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* TYPES_H_ */
