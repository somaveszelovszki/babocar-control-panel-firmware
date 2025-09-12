#include <DebugMessage.hpp>
#include <RaceTrackController.hpp>
#include <cfg_board.hpp>
#include <cmath>
#include <etl/circular_buffer.h>
#include <etl/string.h>
#include <globals.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/ParamManager.hpp>
#include <micro/debug/TaskMonitor.hpp>
#include <micro/log/log.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/timer.hpp>
#include <optional>

using namespace micro;

namespace {

constexpr uint32_t MAX_BUFFER_SIZE = 128;

struct rxBuffer_t {
    char value[MAX_BUFFER_SIZE];
};
rxBuffer_t rxBuffer;
micro::mutex_t incomingMessagesMutex;
etl::circular_buffer<rxBuffer_t, 2> incomingMessages;

char txBuffer[MAX_BUFFER_SIZE];
semaphore_t txSemaphore;
std::tuple<CarProps, ControlData> carData;
LapControlParameters lapControl;
LapControlParameters prevLapControl;
ParamManager::Values params;

void transmit() {
    uart_transmit(uart_Debug, reinterpret_cast<uint8_t*>(txBuffer), etl::strlen(txBuffer));
    txSemaphore.take();
}

bool handleIncomingParam(char* const input) {
    std::optional<ParamManager::NamedParam> param;
    if (!DebugMessage::parse(input, param)) {
        return false;
    }

    if (param) {
        if (globalParams.update(param->first, param->second)) {
            params[param->first] = param->second;
        }
    } else {
        globalParams.getAll(params);
    }

    return true;
}

bool handleIncomingSectionControl(char* const input) {
    std::optional<IndexedSectionControlParameters> sectionControl;
    if (!DebugMessage::parse(input, sectionControl)) {
        return false;
    }

    if (sectionControl) {
        sectionControlOverrideQueue.send(*sectionControl, millisecond_t(0));
    } else {
        lapControlQueue.peek(lapControl, millisecond_t(0));
    }

    return true;
}

bool handleIncomingRadioCommand(char* const input) {
    DebugMessage::RadioCommand command;
    if (!DebugMessage::parse(input, command)) {
        return false;
    }

    radioCommandQueue.send(command.text, millisecond_t(0));
    return true;
}

bool handleIncomingMessages() {
    auto incomingBuffer = []() -> std::optional<rxBuffer_t> {
        std::scoped_lock lock{incomingMessagesMutex};
        if (incomingMessages.empty()) {
            return std::nullopt;
        }

        const auto incomingBuffer = incomingMessages.front();
        incomingMessages.pop();
        return incomingBuffer;
    }();

    if (!incomingBuffer) {
        return false;
    }

    return handleIncomingParam(incomingBuffer->value) ||
           handleIncomingSectionControl(incomingBuffer->value) ||
           handleIncomingRadioCommand(incomingBuffer->value);
}

} // namespace

extern "C" void runDebugTask(void) {
    taskMonitor.registerInitializedTask();

    uart_receive(uart_Debug, reinterpret_cast<uint8_t*>(rxBuffer.value), MAX_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led);
    Timer carPropsSendTimer(millisecond_t(50));
    Timer paramsSyncTimer(millisecond_t(25));
    Timer lapControlCheckTimer(second_t(1));

    globalParams.getAll(params);

    while (true) {
        handleIncomingMessages();

        if (carPropsSendTimer.checkTimeout()) {
            DebugMessage::CarData carData;
            carPropsQueue.peek(carData.props, millisecond_t(0));
            lastControlQueue.peek(carData.control, millisecond_t(0));

            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, carData);
            transmit();
        }

        if (params.empty() && paramsSyncTimer.checkTimeout()) {
            globalParams.sync(params);
        }

        if (!params.empty()) {
            const auto lastElement = std::next(params.begin(), params.size() - 1);
            const ParamManager::NamedParam param{lastElement->first, lastElement->second};
            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, param);
            params.erase(lastElement);
            transmit();
        }

        if (Log::instance().receive(reinterpret_cast<Log::Message&>(txBuffer))) {
            transmit();
        }

        if (lapControl.empty() && lapControlCheckTimer.checkTimeout()) {
            lapControlQueue.peek(lapControl, millisecond_t(0));
            if (lapControl == prevLapControl) {
                lapControl.clear();
            } else {
                prevLapControl = lapControl;
            }
        }

        // sends lap control parameters one by one
        if (!lapControl.empty()) {
            const auto lastIndex   = static_cast<uint8_t>(lapControl.size() - 1);
            const auto lastElement = std::next(lapControl.begin(), lastIndex);
            const IndexedSectionControlParameters sectionControl{lastIndex, *lastElement};
            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, sectionControl);
            lapControl.erase(lastElement);
            transmit();
        }

        taskMonitor.notify(true);
        debugLed.update(taskMonitor.ok());
        os_sleep(millisecond_t(1));
    }
}

void micro_Command_Uart_RxCpltCallback() {
    if (uart_Debug.handle->hdmarx->Instance->NDTR < MAX_BUFFER_SIZE) {
        std::scoped_lock lock{incomingMessagesMutex};
        incomingMessages.push(rxBuffer);
    }

    uart_receive(uart_Debug, reinterpret_cast<uint8_t*>(rxBuffer.value), MAX_BUFFER_SIZE);
}

void micro_Command_Uart_TxCpltCallback() {
    txSemaphore.give();
}
