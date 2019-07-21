#pragma once

#include <micro/bsp/queue.hpp>
#include <micro/bsp/mutex.hpp>
#include <micro/bsp/task.hpp>

namespace cfg {

micro::queue_handle_t * const  queue_Log               = micro::getQueueHandle(micro::QUEUE::LOG);              // LogQueue handle.

micro::mutex_handle_t * const  mutex_Car               = micro::getMutexHandle(micro::MUTEX::CAR);              // CarMutex handle.
micro::mutex_handle_t * const  mutex_FrontLinePos      = micro::getMutexHandle(micro::MUTEX::FRONT_LINE_POS);   // FrontLinePositionsMutex handle.
micro::mutex_handle_t * const  mutex_RearLinePos       = micro::getMutexHandle(micro::MUTEX::REAR_LINE_POS);    // RearLinePositionsMutex handle.

micro::task_handle_t * const   task_Idle               = micro::getTaskHandle(micro::TASK::IDLE_TASK);          // IdleTask handle
micro::task_handle_t * const   task_Control            = micro::getTaskHandle(micro::TASK::CONTROL_TASK);       // ControlTask handle
micro::task_handle_t * const   task_Debug              = micro::getTaskHandle(micro::TASK::DEBUG_TASK);         // DebugTask handle
micro::task_handle_t * const   task_Setup              = micro::getTaskHandle(micro::TASK::SETUP_TASK);         // SetupTask handle

} // namespace cfg


