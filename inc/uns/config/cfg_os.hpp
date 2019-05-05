#pragma once

#include <uns/bsp/queue.hpp>
#include <uns/bsp/mutex.hpp>
#include <uns/bsp/task.hpp>

#define OS_FREERTOS     // OS: FreeRTOS.

namespace uns {
namespace cfg {

queue_handle_t * const  queue_Log               = uns::getQueueHandle(QUEUE::LOG);              // LogQueue handle.

mutex_handle_t * const  mutex_TaskConfig        = uns::getMutexHandle(MUTEX::TASK_CONFIG) ;     // TaskConfigMutex handle.
mutex_handle_t * const  mutex_Car               = uns::getMutexHandle(MUTEX::CAR);              // CarMutex handle.
mutex_handle_t * const  mutex_TargetSpeed       = uns::getMutexHandle(MUTEX::TARGET_SPEED);     // TargetSpeedMutex handle.
mutex_handle_t * const  mutex_FrontLinePos      = uns::getMutexHandle(MUTEX::FRONT_LINE_POS);   // FrontLinePositionsMutex handle.
mutex_handle_t * const  mutex_RearLinePos       = uns::getMutexHandle(MUTEX::REAR_LINE_POS);    // RearLinePositionsMutex handle.

task_handle_t * const   task_Idle               = uns::getTaskHandle(TASK::IDLE_TASK);          // IdleTask handle
task_handle_t * const   task_Control            = uns::getTaskHandle(TASK::CONTROL_TASK);       // ControlTask handle
task_handle_t * const   task_Debug              = uns::getTaskHandle(TASK::DEBUG_TASK);         // DebugTask handle
task_handle_t * const   task_Setup              = uns::getTaskHandle(TASK::SETUP_TASK);         // SetupTask handle

} // namespace cfg
} // namespace uns


