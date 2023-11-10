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
char txBuffer[MAX_BUFFER_SIZE];
semaphore_t txSemaphore;
std::tuple<CarProps, ControlData> carData;
LapControlParameters lapControl;

void transmit(const char * const data) {
    uart_transmit(uart_Debug, reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
    txSemaphore.take();
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
//                   const auto outParams = params.empty()
//                       ? globalParams.getAll()
//                       : globalParams.update(params);
//                   if (!outParams.empty()) {
//                       DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, outParams);
//                   }
               },

               [](const IndexedSectionControlParameters& sectionControl){
                   sectionControlOverrideQueue.send(sectionControl);
               }
           }, *data);
       }

        if (carPropsSendTimer.checkTimeout()) {
            auto& [car, controlData] = carData;
            carPropsQueue.peek(car, millisecond_t(0));
            lastControlQueue.peek(controlData, millisecond_t(0));

            DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, carData);
            transmit(txBuffer);
        }

//       if (const auto changedParams = globalParams.sync(); !changedParams.empty()) {
//           DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, changedParams);
//           transmit(txBuffer);
//       }

       if (Log::instance().receive(reinterpret_cast<Log::Message&>(txBuffer))) {
           transmit(txBuffer);
       }

       if (lapControl.empty()) {
           lapControlQueue.receive(lapControl, millisecond_t(0));
       }

       // sends lap control parameters one by one
       if (!lapControl.empty()) {
           const auto lastIndex = static_cast<uint8_t>(lapControl.size() - 1);
           const auto lastElement = std::next(lapControl.begin(), lastIndex);
           const IndexedSectionControlParameters sectionControl{lastIndex, *lastElement};
           DebugMessage::format(txBuffer, MAX_BUFFER_SIZE, sectionControl);
           lapControl.erase(lastElement);
           transmit(txBuffer);
       }

        SystemManager::instance().notify(true);
        debugLed.update(SystemManager::instance().ok());
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
