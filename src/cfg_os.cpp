#include <cfg_os.hpp>

#include <FreeRTOS.h>
#include <cmsis_os.h>

extern "C" osThreadId IdleTaskHandle;
extern "C" osThreadId ControlTaskHandle;
extern "C" osThreadId DebugTaskHandle;
extern "C" osThreadId SetupTaskHandle;
extern "C" osMessageQId LogQueueHandle;
extern "C" osMutexId CarMutexHandle;
extern "C" osMutexId FrontLinePositionsMutexHandle;
extern "C" osMutexId RearLinePositionsMutexHandle;

namespace cfg {

const micro::queue_handle_t queue_Log = { LogQueueHandle };

const micro::mutex_handle_t mutex_Car          = { CarMutexHandle };
const micro::mutex_handle_t mutex_FrontLinePos = { FrontLinePositionsMutexHandle };
const micro::mutex_handle_t mutex_RearLinePos  = { RearLinePositionsMutexHandle };

const micro::task_handle_t task_Idle    = { IdleTaskHandle };
const micro::task_handle_t task_Control = { ControlTaskHandle };
const micro::task_handle_t task_Debug   = { DebugTaskHandle };
const micro::task_handle_t task_Setup   = { SetupTaskHandle };

}
