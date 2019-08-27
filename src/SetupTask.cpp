#include <micro/bsp/tim.hpp>
#include <micro/utils/log.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <cfg_os.hpp>

#include <globals.hpp>

using namespace micro;

extern LineDetectPanel frontLineDetectPanel;
extern LineDetectPanel rearLineDetectPanel;
extern MotorPanel motorPanel;

namespace {

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

void waitStartSignal() {
    UART_Receive_DMA(cfg::uart_RadioModule, startCounterBuffer, 1);

    while(globals::startSignalEnabled && startCounter != '0') {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        nonBlockingDelay(millisecond_t(50));
    }

    UART_Stop_DMA(cfg::uart_RadioModule);
}

} // namespace

extern "C" void runSetupTask(const void *argument) {
    taskSuspend(cfg::task_Control);
    blockingDelay(millisecond_t(200));     // gives time to auxiliary panels to wake up

    waitStartSignal();

    LOG_DEBUG("Starting panel initialization");

    if (!isOk(motorPanel.start(globals::useSafetyEnableSignal))) {
        LOG_ERROR("motorPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(frontLineDetectPanel.start())) {
        LOG_ERROR("frontLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    if (!isOk(rearLineDetectPanel.start())) {
        LOG_ERROR("rearLineDetectPanel.start failed");
        task::setErrorFlag();
    }

    taskDeleteCurrent();
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    const uint8_t cntr = static_cast<uint8_t>(startCounterBuffer[0]);
    if (cntr == startCounter - 1) {
        startCounter = cntr;
    }
}
