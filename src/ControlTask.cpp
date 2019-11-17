#include <cfg_board.h>
#include <micro/task/common.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/LineController.hpp>
#include <micro/control/PD_Controller.hpp>
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

static Timer motorPanelSendTimer;

bool isFastSpeedSafe(const Line& line) {
    static constexpr millimeter_t MAX_LINE_POS = centimeter_t(8.5f);
    static constexpr radian_t MAX_LINE_ANGLE = degree_t(8.0f);

    return abs(line.pos_front) <= MAX_LINE_POS && abs(line.angle) <= MAX_LINE_ANGLE;
}

void fillMotorPanelData(motorPanelDataIn_t& panelData, m_per_sec_t targetSpeed) {
    panelData.controller_Kc    = globals::motorController_Kc;
    panelData.targetSpeed_mmps = static_cast<int16_t>(static_cast<mm_per_sec_t>(targetSpeed).get());
    panelData.controller_Ti_us = static_cast<microsecond_t>(globals::motorController_Ti).get();

    panelData.flags = 0x00;
    if (globals::useSafetyEnableSignal) panelData.flags |= MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL;
}

} // namespace

extern "C" void runControlTask(const void *argument) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontSteeringServo.positionMiddle();
    rearSteeringServo.positionMiddle();

    motorPanel.start();
    motorPanelSendTimer.start(millisecond_t(20));

    ControlData controlData;
    millisecond_t lastControlDataRecvTime = millisecond_t::ZERO();

    PD_Controller lineController(globals::frontLineController_P, globals::frontLineController_D,
            static_cast<degree_t>(-cfg::WHEEL_MAX_DELTA_FRONT).get(), static_cast<degree_t>(cfg::WHEEL_MAX_DELTA_FRONT).get());

    globals::isControlTaskInitialized = true;

    while (true) {
        if (motorPanel.hasNewValue()) {
            motorPanelDataOut_t motorPanelData = motorPanel.acquireLastValue();
            globals::car.distance = millimeter_t(motorPanelData.distance_mm);
            globals::car.speed = mm_per_sec_t(motorPanelData.actualSpeed_mmps);
            const m_per_sec_t currentTargetSpeed = mm_per_sec_t(motorPanelData.targetSpeed_mmps);

            LOG_DEBUG("actual speed: %f m/s\ttarget speed: %f m/s\tpos: %fmm",
                globals::car.speed.get(),
                currentTargetSpeed.get(),
                static_cast<millimeter_t>(globals::car.distance).get());
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            lastControlDataRecvTime = micro::getTime();

            if (globals::lineFollowEnabled) {
                lineController.setParams(globals::frontLineController_P, globals::frontLineController_D);
                lineController.run(static_cast<centimeter_t>(controlData.baseline.pos_front).get());
                frontSteeringServo.writeWheelAngle(degree_t(lineController.getOutput()));
                rearSteeringServo.writeWheelAngle(degree_t(-lineController.getOutput()));
            }

        } else if (micro::getTime() - lastControlDataRecvTime > millisecond_t(20)) {
            controlData.speed = m_per_sec_t::ZERO();
        }

        if (motorPanelSendTimer.checkTimeout()) {
            motorPanelDataIn_t data;
            //fillMotorPanelData(data, controlData.speed);
            fillMotorPanelData(data, m_per_sec_t(0.35f)); // 35% PWM - for now!
            motorPanel.send(data);
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
