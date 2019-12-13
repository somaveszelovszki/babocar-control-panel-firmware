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

hw::SteeringServo frontSteeringServo(tim_SteeringServo, tim_chnl_FrontServo, cfg::FRONT_SERVO_OFFSET, cfg::FRONT_SERVO_WHEEL_MAX_DELTA, cfg::FRONT_SERVO_WHEEL_TR);
hw::SteeringServo rearSteeringServo(tim_SteeringServo, tim_chnl_RearServo, cfg::REAR_SERVO_OFFSET, cfg::REAR_SERVO_WHEEL_MAX_DELTA, cfg::REAR_SERVO_WHEEL_TR);

hw::Servo frontDistServo(tim_ServoX, tim_chnl_ServoX1, cfg::DIST_SERVO_OFFSET, cfg::DIST_SERVO_MAX_DELTA);

static Timer motorPanelSendTimer;
static Timer frontDistServoUpdateTimer;

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

    vTaskDelay(10); // gives time to other tasks to wake up

    frontSteeringServo.writeWheelAngle(radian_t::zero());
    rearSteeringServo.writeWheelAngle(radian_t::zero());
    frontDistServo.write(radian_t::zero());

    motorPanel.start();
    motorPanel.waitResponse();
    motorPanelSendTimer.start(millisecond_t(20));
    frontDistServoUpdateTimer.start(millisecond_t(20));

    ControlData controlData;
    millisecond_t lastControlDataRecvTime = millisecond_t::zero();

    PD_Controller lineController(globals::frontLineController_P_slow, globals::frontLineController_D_slow,
        static_cast<degree_t>(-cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get());

    //LineController2 lineController2(cfg::CAR_PIVOT_DIST_MID, cfg::OPTO_SENSOR_FRONT_WHEEL_DIST);

    globals::isControlTaskInitialized = true;

    while (true) {
        if (motorPanel.hasNewValue()) {
            motorPanelDataOut_t motorPanelData = motorPanel.acquireLastValue();
            globals::car.distance = millimeter_t(motorPanelData.distance_mm);
            globals::car.speed = mm_per_sec_t(motorPanelData.actualSpeed_mmps);
            const m_per_sec_t currentTargetSpeed = mm_per_sec_t(motorPanelData.targetSpeed_mmps);

//            LOG_DEBUG("actual speed: %f m/s\ttarget speed: %f m/s\tisMotorEnabled:%s\tpos: %fmm",
//                globals::car.speed.get(),
//                currentTargetSpeed.get(),
//                motorPanelData.isMotorEnabled ? "true" : "false",
//                static_cast<millimeter_t>(globals::car.distance).get());
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            lastControlDataRecvTime = micro::getTime();

            if (globals::lineFollowEnabled) {

                if (abs(globals::car.speed) > m_per_sec_t(2.0)) {
                    lineController.setParams(globals::frontLineController_P_fast, globals::frontLineController_D_fast);
                } else {
                    lineController.setParams(globals::frontLineController_P_slow, globals::frontLineController_D_slow);
                }

//                const radian_t steerAngle = lineController2.GetControlSignal(globals::car.speed, controlData.baseline);
//                frontSteeringServo.writeWheelAngle(steerAngle);
//                rearSteeringServo.writeWheelAngle(-steerAngle);

                lineController.run(static_cast<centimeter_t>(controlData.baseline.pos_front - controlData.offset).get());
                frontSteeringServo.writeWheelAngle(controlData.angle + degree_t(lineController.getOutput()));
                rearSteeringServo.writeWheelAngle(controlData.angle + degree_t(-lineController.getOutput()));
            }

        } else if (micro::getTime() - lastControlDataRecvTime > millisecond_t(20)) {
            controlData.speed = m_per_sec_t::zero();
        }

        if (motorPanelSendTimer.checkTimeout()) {
            motorPanelDataIn_t data;

            m_per_sec_t targetSpeed = m_per_sec_t(0);
            if (globals::targetSpeedOverrideActive) {
                targetSpeed = globals::targetSpeedOverride;
            } else {
                targetSpeed = controlData.speed;
            }

            fillMotorPanelData(data, targetSpeed);
            //fillMotorPanelData(data, m_per_sec_t(1.0f));
            //fillMotorPanelData(data, globals::targetSpeedOverride);
            motorPanel.send(data);
        }

        if (frontDistServoUpdateTimer.checkTimeout()) {
            frontDistServo.write(frontSteeringServo.wheelAngle() * globals::frontDistServoAngleWheelTf);
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
