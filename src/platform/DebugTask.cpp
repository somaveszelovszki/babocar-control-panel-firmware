#include <cfg_board.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/timer.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;
extern queue_t<ControlData, 1> lastControlQueue;

Params globalParams('P');
Params raceTrackParams('R');

namespace {

#define FAILING_TASKS_LOG_ENABLED false

constexpr uint32_t MAX_PARAMS_BUFFER_SIZE = 1024;

typedef uint8_t rxParams_t[MAX_PARAMS_BUFFER_SIZE];
ring_buffer<rxParams_t, 3> rxBuffer;
Log::message_t txLog;
char paramsStr[MAX_PARAMS_BUFFER_SIZE];
semaphore_t txSemaphore;

void transmit(const char * const data) {
    uart_transmit(uart_Debug, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take();
}

bool monitorTasks() {
    static Timer failureLogTimer(millisecond_t(100));

    const SystemManager::TaskStates failingTasks = SystemManager::instance().failingTasks();

#if FAILING_TASKS_LOG_ENABLED

    if (failingTasks.size() && failureLogTimer.checkTimeout()) {
        char msg[LOG_MSG_MAX_SIZE];
        uint32_t idx = 0;
        for (SystemManager::TaskStates::const_iterator it = failingTasks.begin(); it != failingTasks.end(); ++it) {
            idx += strncpy_until(&msg[idx], it->details.pcTaskName, min(static_cast<uint32_t>(configMAX_TASK_NAME_LEN), LOG_MSG_MAX_SIZE - idx));
            if (it != failingTasks.back()) {
                idx += strncpy_until(&msg[idx], ", ", sizeof(", "), LOG_MSG_MAX_SIZE - idx);
            }
        }
        msg[idx] = '\0';
        LOG_ERROR("Failing tasks: %s", msg);
    }

#endif // FAILING_TASKS_LOG_ENABLED

    return failingTasks.size() == 0;
}

void serializeCar(const CarProps& car, const ControlData& controlData, char * const str, const uint32_t size) {
    sprint(str, size, "C:%d,%d,%f,%f,%f,%f,%d,%f,%d,%f,%d",
           static_cast<millimeter_t>(car.pose.pos.X).get(),
           static_cast<millimeter_t>(car.pose.pos.Y).get(),
           car.pose.angle.get(),
           car.speed.get(),
           car.frontWheelAngle.get(),
           car.rearWheelAngle.get(),
           static_cast<millimeter_t>(controlData.lineControl.actual.pos).get(),
           controlData.lineControl.actual.angle,
           static_cast<millimeter_t>(controlData.lineControl.target.pos).get(),
           controlData.lineControl.target.angle,
           car.isRemoteControlled ? 1 : 0);
}

} // namespace

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_PARAMS_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led);
    Timer carPropsSendTimer(millisecond_t(50));

    while (true) {
        const rxParams_t *inCmd = rxBuffer.startRead();
        if (inCmd) {
            globalParams.deserializeAll(reinterpret_cast<const char*>(*inCmd), MAX_PARAMS_BUFFER_SIZE);
            raceTrackParams.deserializeAll(reinterpret_cast<const char*>(*inCmd), MAX_PARAMS_BUFFER_SIZE);
            rxBuffer.finishRead();
        }

        if (carPropsSendTimer.checkTimeout()) {
            CarProps car;
            ControlData controlData;
            carPropsQueue.peek(car, millisecond_t(0));
            lastControlQueue.peek(controlData, millisecond_t(0));

            serializeCar(car, controlData, paramsStr, MAX_PARAMS_BUFFER_SIZE);
            transmit(paramsStr);
        }

        if (globalParams.serializeAll(paramsStr, MAX_PARAMS_BUFFER_SIZE) > 0) {
            transmit(paramsStr);
        }

        if (raceTrackParams.serializeAll(paramsStr, MAX_PARAMS_BUFFER_SIZE) > 0) {
            transmit(paramsStr);
        }

        if (Log::instance().receive(txLog)) {
            transmit(txLog);
        }

        debugLed.update(monitorTasks());
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(1));
    }
}

void micro_Command_Uart_RxCpltCallback() {
    if (MAX_PARAMS_BUFFER_SIZE > uart_Debug.handle->hdmarx->Instance->NDTR) {
        rxBuffer.finishWrite();
    }
    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_PARAMS_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
