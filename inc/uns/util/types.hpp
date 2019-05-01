#pragma once

#include <uns/util/types.h>
#include <type_traits>

namespace uns {

typedef float float32_t;
typedef double float64_t;

/**
 * @brief Defines pin states.
 */
enum class PinState : uint8_t {
    RESET = 0,      // Reset state.
    SET             // Set state.
};

/**
 * @brief Status for operations
 */
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

/**
 * @brief Checks is status is ok.
 * @param status
 * @return True if status is ok.
 */
inline bool isOk(Status status) {
    return status == Status::OK;
}

/** @brief Gets status as string.
 * @param status The status to convert to string.
 * @returns The status as string.
 */
const char* getStatusString(Status status);

/**
 * @brief Defines rotation direction.
 */
enum class RotationDir : int32_t {
    LEFT  = 1,
    CENTER = 0,
    RIGHT = -1
};

/**
 * @brief Defines bit order.
 */
enum class BitOrder : uint8_t {
	LITTLE_ENDIAN_ = 0,
	BIG_ENDIAN_
};

/**
 * @brief Defines sign of a number;
 */
enum class Sign : int32_t {
    POSITIVE = 1,
    NEGATIVE = -1
};

template < template <typename...> class base,typename derived>
struct is_base_of_template_impl
{
    template<typename... Ts>
    static constexpr std::true_type  test(const base<Ts...> *);
    static constexpr std::false_type test(...);
    using type = decltype(test(std::declval<derived*>()));
};

template < template <typename...> class base,typename derived>
using is_base_of_template = typename is_base_of_template_impl<base,derived>::type;
} // namespace uns
