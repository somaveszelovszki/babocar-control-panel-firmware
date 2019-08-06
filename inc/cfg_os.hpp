#pragma once

#include <micro/bsp/queue.hpp>
#include <micro/bsp/mutex.hpp>
#include <micro/bsp/task.hpp>

namespace cfg {

extern const micro::queue_handle_t queue_Log;

extern const micro::mutex_handle_t mutex_Car;
extern const micro::mutex_handle_t mutex_FrontLinePos;
extern const micro::mutex_handle_t mutex_RearLinePos;

extern const micro::task_handle_t task_Idle;
extern const micro::task_handle_t task_Control;
extern const micro::task_handle_t task_Command;
extern const micro::task_handle_t task_Setup;

} // namespace cfg
