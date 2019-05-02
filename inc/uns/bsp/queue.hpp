#pragma once

#include <uns/util/units.hpp>

namespace uns {

typedef void* queue_handle_t;    // Queue handle - type is OS library-dependent.

/* @brief Queue instance ids.
 **/
enum class QUEUE : uint8_t {
    LOG,            // LogQueue
    CONTROL_PROPS   // ControlPropsQueue
};

/* @brief Gets queue handle by id.
 * @param queue The queue id.
 * @returns Pointer to the correspondent queue handle.
 **/
queue_handle_t* getQueueHandle(QUEUE queue);

/* @brief Puts content of buffer to the end of the queue if it is not full.
 * @param hQueue Pointer to the queue handle.
 * @param txBuffer The buffer storing the data to send.
 * @return Status indicating operation success (if not OK, it means the buffer is full).
 **/
Status queueSend(queue_handle_t *hQueue, const void *txBuffer);

/* @brief Gets oldest element from the queue, if any.
 * @param hQueue Pointer to the queue handle.
 * @param rxBuffer The buffer that will store the received data.
 * @return Status indicating operation success (if not OK, it means the buffer is empty).
 **/
Status queueReceive(queue_handle_t *hQueue, void *rxBuffer);

} // namespace uns
