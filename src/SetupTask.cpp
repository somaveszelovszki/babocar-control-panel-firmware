#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/LineDetectPanelData.h>
#include <micro/panel/MotorPanel.hpp>
#include <micro/panel/MotorPanelData.h>
#include <micro/task/common.hpp>

#include <cfg_car.hpp>

#include <globals.hpp>

#include <micro/hw/MPU9250_Gyroscope.hpp>

using namespace micro;

extern osThreadId ControlTaskHandle;
extern osThreadId SensorTaskHandle;

#define LOG_QUEUE_LENGTH 16
extern QueueHandle_t logQueue;
static uint8_t logQueueStorageBuffer[LOG_QUEUE_LENGTH * LOG_MSG_MAX_SIZE];
static StaticQueue_t logQueueBuffer;

extern LineDetectPanel frontLineDetectPanel;
extern LineDetectPanel rearLineDetectPanel;
extern MotorPanel motorPanel;
extern hw::MPU9250 gyro;

namespace {

char startCounter = '6';   // start counter will count back from 5 to 0

void waitPanels(void) {
    millisecond_t lastRecvTime = millisecond_t(0);
    do {
        vTaskDelay(10);
        motorPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));

    lastRecvTime = millisecond_t(0);
    do {
        vTaskDelay(10);
        frontLineDetectPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));

    lastRecvTime = millisecond_t(0);
    do {
        vTaskDelay(10);
        rearLineDetectPanel.getLastValue(&lastRecvTime);
    } while(isZero(lastRecvTime));
}

void waitStartSignal(void) {
    HAL_UART_Receive_DMA(uart_RadioModule, reinterpret_cast<uint8_t*>(&startCounter), 1);

    while(globals::startSignalEnabled && startCounter != '0') {
        LOG_DEBUG("Seconds until start: %c", startCounter);
        vTaskDelay(50);
    }

    HAL_UART_DMAStop(uart_RadioModule);
}

} // namespace

extern "C" void runSetupTask(const void *argument) {

    logQueue = xQueueCreateStatic(LOG_QUEUE_LENGTH, LOG_MSG_MAX_SIZE, logQueueStorageBuffer, &logQueueBuffer);

    LOG_DEBUG("SetupTask running...");

    //vTaskSuspend(ControlTaskHandle);
    //vTaskSuspend(SensorTaskHandle);

    vTaskDelay(200);     // gives time to auxiliary panels to wake up

    LOG_DEBUG("Starting initialization...");

    gyro.initialize();

    while(1) {vTaskDelay(1);}

//    motorPanelDataIn_t motorPanelStartData;
//    motorPanelStartData.targetSpeed_mmps = 0;
//    motorPanelStartData.controller_Ti_us = globals::motorController_Ti.get();
//    motorPanelStartData.controller_Kc    = globals::motorController_Kc;
//    motorPanelStartData.flags            = globals::useSafetyEnableSignal ? MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL : 0x00;
//
//    lineDetectPanelDataIn_t frontLineDetectPanelStartData;
//    frontLineDetectPanelStartData.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;
//
//    lineDetectPanelDataIn_t rearLineDetectPanelStartData;
//    rearLineDetectPanelStartData.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;
//
//    motorPanel.start(motorPanelStartData);
//    frontLineDetectPanel.start(frontLineDetectPanelStartData);
//    rearLineDetectPanel.start(rearLineDetectPanelStartData);
//
//    waitPanels();
//    waitStartSignal();

    //xTaskResumeAll();
    vTaskDelete(nullptr);
}
