#pragma once

#include <uns/util/units.hpp>

namespace uns {

typedef void mutex_handle_t;   // Mutex handle - type is OS library-dependent.

/* @brief Mutex instance ids.
 **/
enum class MUTEX : uint8_t {
    CONTROL_PROPS   // ControlPropsMutex
};

/* @brief Gets mutex handle by id.
 * @param mutex The mutex id.
 * @returns Pointer to the correspondent mutex handle.
 **/
mutex_handle_t* getMutexHandle(MUTEX mutex);

/* @brief Waits for mutex to become available and reserves it. Polls while blocking.
 * @param hMutex Pointer to the mutex handle.
 * @param The timeout to wait for the mutex.
 * @return Status indicating operation success (if not OK, it means the mutex is not available).
 **/
Status mutexTake(mutex_handle_t *hMutex, millisecond_t timeout);

/* @brief Checks if mutex is available and reserves it. Does not block.
 * @note This function must be called from an interrupt service routine!
 * @param hMutex Pointer to the mutex handle.
 * @return Status indicating operation success (if not OK, it means the mutex is not available).
 **/
Status mutexTake_ISR(mutex_handle_t *hMutex);

/* @brief Releases mutex.
 * @param hMutex Pointer to the mutex handle.
 * @return Status indicating operation success.
 **/
Status mutexRelease(mutex_handle_t *hMutex);

/* @brief Releases mutex.
 * @note This function must be called from an interrupt service routine!
 * @param hMutex Pointer to the mutex handle.
 * @return Status indicating operation success.
 **/
Status mutexRelease_ISR(mutex_handle_t *hMutex);

} // namespace uns
