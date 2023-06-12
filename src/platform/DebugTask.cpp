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

#include <cfg_board.hpp>
#include <Distances.hpp>
#include <globals.hpp>
#include <RaceTrackController.hpp>

#include <cmath>
#include <optional>

using namespace micro;

namespace {

#define FAILING_TASKS_LOG_ENABLED false

constexpr uint32_t MAX_BUFFER_SIZE = 1024;

typedef uint8_t rxParams_t[MAX_BUFFER_SIZE];
ring_buffer<rxParams_t, 3> rxBuffer;
Log::message_t txLog;
char txBuffer[MAX_BUFFER_SIZE];
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
    sprint(str, size, "C:%d,%d,%f,%f,%f,%f,%d,%f,%d,%f,%d%s",
           static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.X).get())),
           static_cast<int32_t>(std::lround(static_cast<millimeter_t>(car.pose.pos.Y).get())),
           car.pose.angle.get(),
           car.speed.get(),
           car.frontWheelAngle.get(),
           car.rearWheelAngle.get(),
           static_cast<int32_t>(std::lround(controlData.lineControl.actual.pos.get())),
           controlData.lineControl.actual.angle.get(),
           static_cast<int32_t>(std::lround(controlData.lineControl.target.pos.get())),
           controlData.lineControl.target.angle.get(),
           car.isRemoteControlled ? 1 : 0,
           LOG_SEPARATOR_SEQ);
}

void serializeTrackSectionControl(const LapControlParameters& lapControl, char * const str, const uint32_t size) {
    uint32_t idx = 0u;
    str[idx++] = 'T';
    str[idx++] = ':';
    str[idx++] = '[';

    for (uint32_t i = 0u; i < lapControl.size(); ++i) {
        const TrackSection::ControlParameters& control = lapControl[i];
        idx += sprint(&str[idx], size - idx, "[%f,%u,%d,%f,%d,%f]%s",
                      control.speed.get(),
                      static_cast<uint32_t>(std::lround(control.rampTime.get())),
                      static_cast<int32_t>(std::lround(control.lineGradient.first.pos.get())),
                      control.lineGradient.first.angle.get(),
                      static_cast<int32_t>(std::lround(control.lineGradient.second.pos.get())),
                      control.lineGradient.second.angle.get(),
                      i < lapControl.size() - 1 ? "," : "");
    }

    str[idx++] = ']';
    strncpy_until(&str[idx], LOG_SEPARATOR_SEQ, size - idx);
}

std::optional<LapControlParameters> deserializeTrackSectionControl(const char * const str, const uint32_t size) {
    if (str[0] != 'T') {
        return std::nullopt;
    }

    LapControlParameters lapControl;

    uint32_t idx = 3; // skips 'T:['

    while (str[idx++] == '[') {
        TrackSection::ControlParameters control;
        float f = 0.0f;
        int32_t i = 0u;

        idx += micro::atof(str, &f) + 1;
        control.speed = m_per_sec_t(f);

        idx += micro::atoi(str, &i) + 1;
        control.rampTime = millisecond_t(i);

        idx += micro::atoi(str, &i) + 1;
        control.lineGradient.first.pos = millimeter_t(i);

        idx += micro::atof(str, &f) + 1;
        control.lineGradient.first.angle = radian_t(f);

        idx += micro::atoi(str, &i) + 1;
        control.lineGradient.second.pos = millimeter_t(i);

        idx += micro::atof(str, &f) + 1;
        control.lineGradient.second.angle = radian_t(f);

        ++idx; // skips ']'
        if (str[idx] == ',') {
            ++idx;
        }

        lapControl.push_back(control);
    }

    return lapControl;
}

} // namespace

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led);
    Timer carPropsSendTimer(millisecond_t(50));

    while (true) {
        const rxParams_t *inCmd = rxBuffer.startRead();
        if (inCmd) {
            globalParams.deserializeAll(reinterpret_cast<const char*>(*inCmd), MAX_BUFFER_SIZE);

            if (const auto lapControl = deserializeTrackSectionControl(reinterpret_cast<const char*>(*inCmd), MAX_BUFFER_SIZE)) {
                lapControlOverrideQueue.overwrite(*lapControl);
            }

            rxBuffer.finishRead();
        }

        if (carPropsSendTimer.checkTimeout()) {
            CarProps car;
            ControlData controlData;
            carPropsQueue.peek(car, millisecond_t(0));
            lastControlQueue.peek(controlData, millisecond_t(0));

            serializeCar(car, controlData, txBuffer, MAX_BUFFER_SIZE);
            transmit(txBuffer);
        }

        if (globalParams.serializeAll(txBuffer, MAX_BUFFER_SIZE) > 0) {
            transmit(txBuffer);
        }

        if (Log::instance().receive(txLog)) {
            transmit(txLog);
        }

        LapControlParameters lapControl;
        if (lapControlQueue.receive(lapControl, millisecond_t(0))) {
            serializeTrackSectionControl(lapControl, txBuffer, MAX_BUFFER_SIZE);
            transmit(txBuffer);
        }

        debugLed.update(monitorTasks());
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(1));
    }
}

void micro_Command_Uart_RxCpltCallback() {
    if (MAX_BUFFER_SIZE > uart_Debug.handle->hdmarx->Instance->NDTR) {
        rxBuffer.finishWrite();
    }
    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
