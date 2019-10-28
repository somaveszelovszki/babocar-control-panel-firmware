#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/LineController.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>

#include <cfg_car.hpp>
#include <globals.hpp>
#include <ControlData.hpp>

#include <FreeRTOS.h>
#include <queue.h>

using namespace micro;

#define CONTROL_QUEUE_LENGTH 1
QueueHandle_t controlQueue;
static uint8_t controlQueueStorageBuffer[CONTROL_QUEUE_LENGTH * sizeof(ControlData)];
static StaticQueue_t controlQueueBuffer;

namespace {

MotorPanel motorPanel(uart_MotorPanel);

hw::SteeringServo frontSteeringServo(tim_SteeringServo, tim_chnl_FrontServo, cfg::SERVO_MID_FRONT, cfg::WHEEL_MAX_DELTA_FRONT, cfg::SERVO_WHEEL_TR_FRONT);
hw::SteeringServo rearSteeringServo(tim_SteeringServo, tim_chnl_RearServo, cfg::SERVO_MID_REAR, cfg::WHEEL_MAX_DELTA_REAR, cfg::SERVO_WHEEL_TR_REAR);

Timer motorPanelSendTimer;

bool isFastSpeedSafe(const Line& line) {
    static constexpr millimeter_t MAX_LINE_POS = centimeter_t(8.5f);
    static constexpr radian_t MAX_LINE_ANGLE = degree_t(8.0f);

    return abs(line.pos_front) <= MAX_LINE_POS && abs(line.angle) <= MAX_LINE_ANGLE;
}

void fillMotorPanelData(m_per_sec_t targetSpeed, motorPanelDataIn_t& panelData) {
    panelData.targetSpeed_mmps = static_cast<int16_t>(static_cast<mm_per_sec_t>(targetSpeed).get());
    panelData.controller_Ti_us = static_cast<microsecond_t>(globals::motorController_Ti).get();
    panelData.controller_Kc    = globals::motorController_Kc;
    panelData.flags            = globals::useSafetyEnableSignal ? MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL : 0x00;
}

} // namespace

extern "C" void runControlTask(const void *argument) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(5); // gives time to other tasks to initialize their queues

    motorPanelDataIn_t motorPanelData;
    fillMotorPanelData(m_per_sec_t(0), motorPanelData);
    motorPanel.start(motorPanelData);
    motorPanel.waitStart();
    motorPanelSendTimer.start(millisecond_t(5));

    ControlData controlData;
    millisecond_t lastControlDataRecvTime = millisecond_t::ZERO();

    while (true) {
        if (motorPanel.hasNewValue()) {
            motorPanelDataOut_t motorPanelData = motorPanel.acquireLastValue();
            globals::car.distance = millimeter_t(motorPanelData.distance_mm);
            globals::car.speed = mm_per_sec_t(motorPanelData.actualSpeed_mmps);
            //LOG_DEBUG("speed: %f m/s", globals::car.speed.get());
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            lastControlDataRecvTime = micro::getTime();
        } else if (micro::getTime() - lastControlDataRecvTime > millisecond_t(20)) {
            controlData.speed = m_per_sec_t::ZERO();
        }

        if (motorPanelSendTimer.checkTimeout()) {
            motorPanelDataIn_t data;
            data.targetSpeed_mmps = static_cast<int16_t>(static_cast<mm_per_sec_t>(controlData.speed).get());
            data.controller_Ti_us = globals::motorController_Ti.get();
            data.controller_Kc    = globals::motorController_Kc;
            data.flags            = globals::useSafetyEnableSignal ? MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL : 0x00;
            motorPanel.send(data);
        }

        if (globals::lineFollowEnabled) {

        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for motor panel UART RxCplt - called when receive finishes.
 */
void micro_MotorPanel_Uart_RxCpltCallback() {
    motorPanel.onDataReceived();
}
