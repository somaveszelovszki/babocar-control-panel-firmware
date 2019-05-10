#include <uns/config/cfg_board.hpp>
#include <uns/config/cfg_os.hpp>

#ifdef OS_FREERTOS

#include <uns/bsp/tim.hpp>
#include <uns/bsp/it.hpp>
#include <uns/bsp/queue.hpp>
#include <uns/bsp/mutex.hpp>
#include <uns/bsp/task.hpp>

#include "cmsis_os.h"

extern "C" osMessageQId LogQueueHandle;

extern "C" osMutexId    TaskConfigMutexHandle;
extern "C" osMutexId    CarMutexHandle;
extern "C" osMutexId    TargetSpeedMutexHandle;
extern "C" osMutexId    FrontLinePositionsMutexHandle;
extern "C" osMutexId    RearLinePositionsMutexHandle;

extern "C" osThreadId   IdleTaskHandle;;
extern "C" osThreadId   ControlTaskHandle;
extern "C" osThreadId   DebugTaskHandle;
extern "C" osThreadId   SetupTaskHandle;

namespace uns {

// TIMER

void nonBlockingDelay(millisecond_t delay) {
    osDelay(delay.get());
}

// QUEUE

queue_handle_t* getQueueHandle(QUEUE queue) {
    queue_handle_t *hQueue = nullptr;
    switch (queue) {
    case QUEUE::LOG:    hQueue = &LogQueueHandle;   break;
    }
    return hQueue;
}

Status queueSend(queue_handle_t * const hQueue, const void * const txBuffer){
    return xQueueSend(static_cast<osMessageQId>(*hQueue), txBuffer, 0) > 0 ? Status::OK : Status::BUFFER_FULL;
}

Status queueReceive(queue_handle_t * const hQueue, void * const rxBuffer){
    return xQueueReceive(static_cast<osMessageQId>(*hQueue), rxBuffer, 0) > 0 ? Status::OK : Status::NO_NEW_DATA;
}

// IT

void enterCritical() {
    taskENTER_CRITICAL();
}

uint32_t enterCritical_ISR() {
    return static_cast<uint32_t>(taskENTER_CRITICAL_FROM_ISR());
}

void exitCritical() {
    taskEXIT_CRITICAL();
}

void exitCritical_ISR(uint32_t uxSavedInterruptStatus) {
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

// MUTEX

mutex_handle_t* getMutexHandle(MUTEX mutex) {
    mutex_handle_t *hMutex = nullptr;
    switch (mutex) {
    case MUTEX::TASK_CONFIG:    hMutex = TaskConfigMutexHandle;         break;
    case MUTEX::CAR:            hMutex = CarMutexHandle;                break;
    case MUTEX::TARGET_SPEED:   hMutex = TargetSpeedMutexHandle;        break;
    case MUTEX::FRONT_LINE_POS: hMutex = FrontLinePositionsMutexHandle; break;
    case MUTEX::REAR_LINE_POS:  hMutex = RearLinePositionsMutexHandle;  break;
    }
    return hMutex;
}

Status mutexTake(mutex_handle_t *hMutex, millisecond_t timeout) {
    return xSemaphoreTake(static_cast<osMutexId>(hMutex), timeout.get()) == pdTRUE ? Status::OK : Status::TIMEOUT;
}

Status mutexTake_ISR(mutex_handle_t *hMutex) {
    return xSemaphoreTakeFromISR(static_cast<osMutexId>(hMutex), nullptr) == pdTRUE ? Status::OK : Status::BUSY;
}

Status mutexRelease(mutex_handle_t *hMutex)  {
    return xSemaphoreGive(static_cast<osMutexId>(hMutex)) == pdTRUE ? Status::OK : Status::ERROR;
}

Status mutexRelease_ISR(mutex_handle_t *hMutex)  {
    return xSemaphoreGiveFromISR(static_cast<osMutexId>(hMutex), nullptr) == pdTRUE ? Status::OK : Status::ERROR;
}

// TASK

task_handle_t* getTaskHandle(TASK task) {
    task_handle_t *hTask = nullptr;
    switch (task) {
    case TASK::IDLE_TASK:       hTask = &IdleTaskHandle;    break;
    case TASK::CONTROL_TASK:    hTask = &ControlTaskHandle; break;
    case TASK::DEBUG_TASK:      hTask = &DebugTaskHandle;   break;
    case TASK::SETUP_TASK:      hTask = &SetupTaskHandle;   break;
    }
    return hTask;
}

void taskResume(task_handle_t *hTask) {
    vTaskResume(static_cast<osThreadId>(hTask));
}

void taskSuspend(task_handle_t *hTask) {
    vTaskSuspend(static_cast<osThreadId>(hTask));
}

void taskDelete(task_handle_t *hTask) {
    vTaskDelete(static_cast<osThreadId>(hTask));
}

void taskDeleteCurrent() {
    vTaskDelete(nullptr);
}

} // namespace uns

#endif // OS_FREERTOS
