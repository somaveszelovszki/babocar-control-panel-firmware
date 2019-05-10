#include <uns/config/cfg_board.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/config/cfg_os.hpp>
#include <uns/task/common.hpp>
#include <uns/util/debug.hpp>
#include <uns/globals.hpp>
#include <uns/panel/LineDetectPanel.hpp>
#include <uns/panel/MotorPanel.hpp>

using namespace uns;

extern panel::LineDetectPanel frontLineDetectPanel;
extern panel::LineDetectPanel rearLineDetectPanel;
extern panel::MotorPanel motorPanel;

namespace {

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

void waitStartSignal() {
    uns::UART_Receive_DMA(cfg::uart_RadioModule, startCounterBuffer, 1);

    while(globals::startSignalEnabled && startCounter != '0') {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        uns::nonBlockingDelay(millisecond_t(50));
    }

    uns::UART_Stop_DMA(cfg::uart_RadioModule);
}

} // namespace

extern "C" void runSetupTask(const void *argument) {
    uns::taskSuspend(cfg::task_Control);
    uns::blockingDelay(millisecond_t(200));     // gives time to auxiliary panels to wake up

    waitStartSignal();

    LOG_DEBUG("Starting panel initialization");

    Status status;

    if (!isOk(status = motorPanel.start(globals::useSafetyEnableSignal))) {
        LOG_ERROR_WITH_STATUS(status, "motorPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(status = frontLineDetectPanel.start())) {
        LOG_ERROR_WITH_STATUS(status, "frontLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(status = rearLineDetectPanel.start())) {
        LOG_ERROR_WITH_STATUS(status, "rearLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    uns::taskDeleteCurrent();
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void uns_RadioModule_Uart_RxCpltCallback() {
    const uint8_t cntr = static_cast<uint8_t>(startCounterBuffer[0]);
    if (cntr == startCounter - 1) {
        startCounter = cntr;
    }
}
