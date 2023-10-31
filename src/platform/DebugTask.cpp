#include <cmath>
#include <optional>
#include <variant>

#include <micro/container/ring_buffer.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/ParamManager.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/log/log.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/variant_utils.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <DebugMessage.hpp>
#include <globals.hpp>
#include <RaceTrackController.hpp>

using namespace micro;

namespace {

constexpr uint32_t MAX_BUFFER_SIZE = 512;

typedef uint8_t rxParams_t[MAX_BUFFER_SIZE];
ring_buffer<rxParams_t, 2> rxBuffer;
Log::Message txLog;
char txBuffer[MAX_BUFFER_SIZE];
semaphore_t txSemaphore;

void transmit(const char * const data) {
    uart_transmit(uart_Debug, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take();
}

bool monitorTasks() {
    const auto failingTasks = SystemManager::instance().failingTasks();

    static Timer failureLogTimer(millisecond_t(500));

    if (failingTasks.size() && failureLogTimer.checkTimeout()) {
        // TODO implement join_to() in format.hpp
        // Log::message_t msg;
        // uint32_t idx = 0;
        // for (size_t i = 0; i < failingTasks.size(); i++) {
        //     idx += strncpy_until(&msg[idx], failingTasks[i].info.name.c_str(), sizeof(Log::message_t) - idx);
        //     if (i < failingTasks.size() - 1) {
        //         idx += strncpy_until(&msg[idx], ", ", sizeof(", "), sizeof(Log::message_t) - idx);
        //     }
        // }
        // msg[idx] = '\0';
        // LOG_ERROR("Failing tasks: %s", msg);
    }

    return failingTasks.empty();
}

} // namespace

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    uart_receive(uart_Debug, *rxBuffer.startWrite(), MAX_BUFFER_SIZE);

    DebugLed debugLed(gpio_Led);
    Timer carPropsSendTimer(millisecond_t(50));

    while (true) {
        const auto data = [inCmd = rxBuffer.startRead()]() -> std::optional<DebugMessage::value_type> {
            if (!inCmd) {
                return std::nullopt;
            }

            const auto d = DebugMessage::parse(const_cast<char*>(reinterpret_cast<const char*>(*inCmd)));
            rxBuffer.finishRead();
            return d;
        }();

        if (data) {
            std::visit(micro::variant_visitor{
                [](const std::tuple<CarProps, ControlData>& v){},

                [](const ParamManager::Values& params){
                    const auto notifyAllParams = params.empty();
                    const auto outParams = globalParams.update(notifyAllParams, params);
                    if (!outParams.empty()) {
                        DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, outParams);
                    }
                },

                [](const LapControlParameters& lapControl){
                    lapControlOverrideQueue.overwrite(lapControl);
                }
            }, *data);
        }

        if (carPropsSendTimer.checkTimeout()) {
            std::tuple<CarProps, ControlData> data;
            auto& [car, controlData] = data;
            carPropsQueue.peek(car, millisecond_t(0));
            lastControlQueue.peek(controlData, millisecond_t(0));

            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, data);
            transmit(txBuffer);
        }

        if (const auto changedParams = globalParams.update(); !changedParams.empty()) {
            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, changedParams);
            transmit(txBuffer);
        }

        if (Log::instance().receive(txLog)) {
            transmit(txLog);
        }

        LapControlParameters lapControl;
        if (lapControlQueue.receive(lapControl, millisecond_t(0))) {
            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, lapControl);
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
