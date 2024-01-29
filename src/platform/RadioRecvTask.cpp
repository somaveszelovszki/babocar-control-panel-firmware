#include <optional>

#include <etl/circular_buffer.h>

#include <micro/debug/TaskMonitor.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/mutex.hpp>
#include <micro/port/task.hpp>
#include <micro/log/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>

using namespace micro;

namespace {

struct rxBuffer_t{ char value[RADIO_COMMAND_MAX_LENGTH]; };
rxBuffer_t rxBuffer;
micro::mutex_t incomingMessagesMutex;
etl::circular_buffer<rxBuffer_t, 4> incomingMessages;

std::optional<etl::string<RADIO_COMMAND_MAX_LENGTH>> read() {
    std::scoped_lock lock{incomingMessagesMutex};
    if (incomingMessages.empty()) {
        return std::nullopt;
    }

    auto incomingBuffer = incomingMessages.front();
    incomingMessages.pop();

    size_t i = 0;
    for (; i < RADIO_COMMAND_MAX_LENGTH - 1 && incomingBuffer.value[i] != '\r'; i++) {}
    incomingBuffer.value[i] = '\0';

    return incomingBuffer.value;
}

} // namespace

extern "C" void runRadioRecvTask(void) {
    taskMonitor.registerInitializedTask();

    uart_receive(uart_RadioModule, reinterpret_cast<uint8_t*>(rxBuffer.value), RADIO_COMMAND_MAX_LENGTH);

    while (true) {
        if (const auto command = read()) {
            LOG_INFO("Received radio command: {}", *command);
            radioCommandQueue.send(*command);
        }

        taskMonitor.notify(true);
        os_sleep(millisecond_t(20));
    }
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    if (uart_RadioModule.handle->hdmarx->Instance->NDTR < RADIO_COMMAND_MAX_LENGTH) {
        std::scoped_lock lock{incomingMessagesMutex};
        incomingMessages.push(rxBuffer);
    }

    uart_receive(uart_RadioModule, reinterpret_cast<uint8_t*>(rxBuffer.value), RADIO_COMMAND_MAX_LENGTH);
}
