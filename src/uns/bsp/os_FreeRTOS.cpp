#include <config/cfg_board.hpp>

#ifdef OS_FREERTOS

#include <uns/bsp/tim.hpp>
#include <uns/bsp/queue.hpp>
#include <uns/bsp/it.hpp>
#include <uns/bsp/mutex.hpp>
#include <uns/bsp/task.hpp>

#include "cmsis_os.h"

extern "C" osMessageQId LogQueueHandle;
extern "C" osMessageQId ControlPropsQueueHandle;

extern "C" osMutexId ControlPropsMutexHandle;

using namespace uns;

// TIMER

void uns::nonBlockingDelay(millisecond_t delay) {
    osDelay(delay.get());
}

// QUEUE

queue_handle_t* uns::getQueueHandle(QUEUE queue) {
    queue_handle_t *hQueue = nullptr;
    switch (queue) {
    case QUEUE::LOG:            hQueue = &LogQueueHandle;           break;
    case QUEUE::CONTROL_PROPS:  hQueue = &ControlPropsQueueHandle;  break;
    }
    return hQueue;
}

Status uns::queueSend(queue_handle_t * const hQueue, const void * const txBuffer){
    Status result = xQueueSend(static_cast<osMessageQId>(*hQueue), txBuffer, 0) > 0 ? Status::OK : Status::BUFFER_FULL;
    return result;
}

Status uns::queueReceive(queue_handle_t * const hQueue, void * const rxBuffer){
    Status result = xQueueReceive(static_cast<osMessageQId>(*hQueue), rxBuffer, 0) > 0 ? Status::OK : Status::NO_NEW_DATA;
    return result;
}

// IT

void uns::enterCritical() {
    taskENTER_CRITICAL();
}

uint32_t uns::enterCritical_ISR() {
    return static_cast<uint32_t>(taskENTER_CRITICAL_FROM_ISR());
}

void uns::exitCritical() {
    taskEXIT_CRITICAL();
}

void uns::exitCritical_ISR(uint32_t uxSavedInterruptStatus) {
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

// MUTEX

mutex_handle_t* uns::getMutexHandle(MUTEX mutex) {
    mutex_handle_t *hMutex = nullptr;
    switch (mutex) {
    case MUTEX::CONTROL_PROPS: hMutex = ControlPropsMutexHandle; break;
    }
    return hMutex;
}

Status uns::mutexTake(mutex_handle_t *hMutex, millisecond_t timeout) {
    Status result = xSemaphoreTake(static_cast<osMutexId>(hMutex), timeout.get()) == pdTRUE ? Status::OK : Status::TIMEOUT;
    return result;
}

Status uns::mutexTake_ISR(mutex_handle_t *hMutex) {
    Status result = xSemaphoreTakeFromISR(static_cast<osMutexId>(hMutex), NULL) == pdTRUE ? Status::OK : Status::BUSY;
    return result;
}

Status uns::mutexRelease(mutex_handle_t *hMutex)  {
    Status result = xSemaphoreGive(static_cast<osMutexId>(hMutex)) == pdTRUE ? Status::OK : Status::ERROR;
    return result;
}

Status uns::mutexRelease_ISR(mutex_handle_t *hMutex)  {
    Status result = xSemaphoreGiveFromISR(static_cast<osMutexId>(hMutex), NULL) == pdTRUE ? Status::OK : Status::ERROR;
    return result;
}

// TASK

void uns::deleteCurrentTask() {
    vTaskDelete(nullptr);
}

#endif // OS_FREERTOS
