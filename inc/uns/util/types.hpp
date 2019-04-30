#pragma once

#include <uns/util/types.h>

namespace uns {

/* @brief Pin states.
 **/
enum class PinState : uint1_t {
    RESET = 0,      // Reset state.
    SET             // Set state.
};

/* @brief Status for operations
 **/
enum class Status : uint32_t {
    OK = 0,         // OK.
    ERROR,          // Unknown error.
    BUSY,           // Resource busy.
    TIMEOUT,        // Waiting has reached timeout.
    INVALID_ID,     // Invalid identifier detected.
    INVALID_DATA,   // Invalid data detected.
    NO_NEW_DATA,    // No new data is available.
    BUFFER_FULL     // Buffer is full.
};

/* @brief Checks if status is OK.
 * @param status The status to check.
 * @returns Boolean value indicating if status is OK.
 **/
bool isOk(Status status);

/* @brief Gets status as string.
 * @param status The status to convert to string.
 * @returns The status as string.
 **/
const char * const getStatusString(Status status);

/* @brief Defines rotation direction.
 **/
enum class RotationDir : int8_t {
    LEFT  = 1,
    CENTER = 0,
    RIGHT = -1
};

/* @brief Defines bit order.
 **/
enum class BitOrder : uint1_t {
	LITTLE_ENDIAN_ = 0,
	BIG_ENDIAN_
};

/* @brief Defines sign of a number;
 **/
enum class Sign : int8_t {
    POSITIVE = 1,
    NEGATIVE = -1
};
} // namespace uns
