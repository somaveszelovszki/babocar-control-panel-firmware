#include <micro/bsp/tim.hpp>
#include <micro/utils/log.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/LineDetectPanelData.h>
#include <micro/panel/MotorPanel.hpp>
#include <micro/panel/MotorPanelData.h>
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

void waitPanels(void) {
    millisecond_t lastRecvTime = millisecond_t(0);
    do {
        nonBlockingDelay(millisecond_t(10));
        motorPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));

    lastRecvTime = millisecond_t(0);
    do {
        nonBlockingDelay(millisecond_t(10));
        frontLineDetectPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));

    lastRecvTime = millisecond_t(0);
    do {
        nonBlockingDelay(millisecond_t(10));
        rearLineDetectPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));
}

void waitStartSignal(void) {
    UART_Receive_DMA(cfg::uart_RadioModule, startCounterBuffer, 1);

    while(globals::startSignalEnabled && startCounter != '0') {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        nonBlockingDelay(millisecond_t(50));
    }

    UART_Stop_DMA(cfg::uart_RadioModule);
}

} // namespace

extern "C" void runSetupTask(const void *argument) {

    taskSuspend(cfg::task_Control); // suspends ControlTask so that it cannot run until initialization finishes

    nonBlockingDelay(millisecond_t(200));     // gives time to auxiliary panels to wake up

    LOG_DEBUG("Starting panel initialization");

    motorPanelDataIn_t motorPanelStartData;
    motorPanelStartData.targetSpeed_mmps = 0;
    motorPanelStartData.controller_Ti_us = globals::motorController_Ti.get();
    motorPanelStartData.controller_Kc    = globals::motorController_Kc;
    motorPanelStartData.flags            = globals::useSafetyEnableSignal ? MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL : 0x00;

    lineDetectPanelDataIn_t frontLineDetectPanelStartData;
    frontLineDetectPanelStartData.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;

    lineDetectPanelDataIn_t rearLineDetectPanelStartData;
    rearLineDetectPanelStartData.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;

    motorPanel.start(motorPanelStartData);
    frontLineDetectPanel.start(frontLineDetectPanelStartData);
    rearLineDetectPanel.start(rearLineDetectPanelStartData);

    waitPanels();
    waitStartSignal();

    taskResumeAll();
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
