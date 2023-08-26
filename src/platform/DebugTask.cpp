#include <cmath>
#include <optional>
#include <variant>

#include <ArduinoJson.hpp>

#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/ParamManager.hpp>
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
#include <debug_protocol.hpp>
#include <Distances.hpp>
#include <globals.hpp>
#include <RaceTrackController.hpp>

using namespace micro;

namespace {

#define FAILING_TASKS_LOG_ENABLED true

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
    const auto failingTasks = SystemManager::instance().failingTasks();

#if FAILING_TASKS_LOG_ENABLED
    static Timer failureLogTimer(millisecond_t(100));

    if (failingTasks.size() && failureLogTimer.checkTimeout()) {
        Log::message_t msg;
        uint32_t idx = 0;
        for (size_t i = 0; i < failingTasks.size(); i++) {
            idx += strncpy_until(&msg[idx], failingTasks[i].info.name.c_str(), sizeof(Log::message_t) - idx);
            if (i < failingTasks.size() - 1) {
                idx += strncpy_until(&msg[idx], ", ", sizeof(", "), sizeof(Log::message_t) - idx);
            }
        }
        msg[idx] = '\0';
        LOG_ERROR("Failing tasks: %s", msg);
    }

#endif // FAILING_TASKS_LOG_ENABLED

    return failingTasks.size() == 0;
}

std::optional<LapControlParameters> deserializeTrackSectionControl(const char * const str, const uint32_t size) {
    if (str[0] != TRACK_CONTROL_PREFIX) {
        return std::nullopt;
    }

    LapControlParameters lapControl;

    uint32_t idx = 3; // skips prefix + ':['

    while (str[idx++] == '[') {
        TrackSection::ControlParameters control;
        char name[sizeof(TrackSection::Name::MAX_SIZE)] = "";
        float f = 0.0f;
        int32_t i = 0u;

        idx += strncpy_until(name, &str[idx], size - idx, ',');

        idx += micro::atof(&str[idx], &f) + 1;
        control.speed = m_per_sec_t(f);

        idx += micro::atoi(&str[idx], &i) + 1;
        control.rampTime = millisecond_t(i);

        idx += micro::atoi(&str[idx], &i) + 1;
        control.lineGradient.first.pos = millimeter_t(i);

        idx += micro::atof(&str[idx], &f) + 1;
        control.lineGradient.first.angle = radian_t(f);

        idx += micro::atoi(&str[idx], &i) + 1;
        control.lineGradient.second.pos = millimeter_t(i);

        idx += micro::atof(&str[idx], &f) + 1;
        control.lineGradient.second.angle = radian_t(f);

        ++idx; // skips ']'
        if (str[idx] == ',') {
            ++idx;
        }

        lapControl.insert({name, control});
    }

    return lapControl;
}

template <typename ...Args>
size_t formatMessage(char * const output, const size_t size, const DebugMessageType type,  Args... args) {
    size_t n = format(output, size, type);
    n += format(&output[n], size - n, std::forward<Args>(args)...);
    n += format(&output[n], size - n, DebugMessageSeparator{});
    return n;
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
            //globalParams.deserializeAll(reinterpret_cast<const char*>(*inCmd), MAX_BUFFER_SIZE);

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

            formatMessage(txBuffer, MAX_BUFFER_SIZE, DebugMessageType::Car, car, controlData);
            transmit(txBuffer);
        }

        if (const auto changedParams = globalParams.update(); !changedParams.empty()) {
            formatMessage(txBuffer, MAX_BUFFER_SIZE, DebugMessageType::Params, changedParams);
            transmit(txBuffer);
        }

        if (Log::instance().receive(txLog)) {
            transmit(txLog);
        }

        LapControlParameters lapControl;
        if (lapControlQueue.receive(lapControl, millisecond_t(0))) {
            formatMessage(txBuffer, MAX_BUFFER_SIZE, DebugMessageType::TrackControl, lapControl);
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
