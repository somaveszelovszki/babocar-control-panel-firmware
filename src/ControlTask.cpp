#include <micro/control/PD_Controller.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/updatable.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <micro/panel/MotorPanelLinkData.hpp>
#include <queue.h>

using namespace micro;

#define CONTROL_QUEUE_LENGTH 1
QueueHandle_t controlQueue;
static uint8_t controlQueueStorageBuffer[CONTROL_QUEUE_LENGTH * sizeof(ControlData)];
static StaticQueue_t controlQueueBuffer;

namespace {

PanelLink<MotorOutPanelLinkData, MotorInPanelLinkData> motorPanelLink(panelLinkRole_t::Master, uart_MotorPanel);

uint16_t servoAngle_to_deg_per4(const radian_t angle, const radian_t offset) {
    return static_cast<uint16_t>(clamp((offset + angle).get() * 4, 0.0f, 1000.0f));
}

radian_t deg_per4_to_servoAngle(const uint16_t angle_deg_per4, const radian_t offset) {
    return degree_t(angle_deg_per4) / 4 - offset;
}

uint16_t wheelAngle_to_deg_per4(const radian_t wheelAngle, const radian_t offset) {
    return servoAngle_to_deg_per4(wheelAngle / cfg::SERVO_WHEEL_TRANSFER_RATE, offset);
}

radian_t deg_per4_to_wheelAngle(const uint16_t servoAngle_deg_per4, const radian_t offset) {
    return deg_per4_to_servoAngle(servoAngle_deg_per4, offset) * cfg::SERVO_WHEEL_TRANSFER_RATE;
}

void fillMotorPanelData(MotorInPanelLinkData& txData, const m_per_sec_t speed, const millisecond_t speedRamp,
    const radian_t frontWheelTargetAngle, const radian_t rearWheelTargetAngle, const radian_t distSensorServoAngle) {
    txData.controller_P                   = globals::motorCtrl_P;
    txData.controller_I                   = globals::motorCtrl_I;
    txData.controller_integral_max        = globals::motorCtrl_integral_max;
    txData.targetSpeed_mmps               = static_cast<int16_t>(static_cast<mm_per_sec_t>(speed).get());
    txData.useSafetyEnableSignal          = globals::useSafetyEnableSignal;
    txData.targetSpeedRampTime_s_per_128  = static_cast<uint16_t>(clamp(static_cast<second_t>(speedRamp).get() * 128, 0.0f, 1000.0f));
    txData.frontServoTargetAngle_deg_per4 = wheelAngle_to_deg_per4(frontWheelTargetAngle, globals::frontSteeringServoOffset);
    txData.rearServoTargetAngle_deg_per4  = wheelAngle_to_deg_per4(rearWheelTargetAngle, globals::rearSteeringServoOffset);
    txData.extraServoTargetAngle_deg_per4 = servoAngle_to_deg_per4(globals::distServoEnabled ? distSensorServoAngle : radian_t(0), cfg::DIST_SERVO_OFFSET);
}

static void parseMotorPanelData(const MotorOutPanelLinkData& rxData) {
    globals::car.speed           = mm_per_sec_t(rxData.speed_mmps);
    globals::car.distance        = millimeter_t(rxData.distance_mm);
    globals::car.frontWheelAngle = deg_per4_to_wheelAngle(rxData.frontServoAngle_deg_per4, globals::frontSteeringServoOffset);
    globals::car.rearWheelAngle  = deg_per4_to_wheelAngle(rxData.rearServoAngle_deg_per4, globals::rearSteeringServoOffset);
}

} // namespace

extern "C" void runControlTask(void) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    MotorOutPanelLinkData rxData;
    MotorInPanelLinkData txData;
    ControlData controlData;

    radian_t frontWheelTargetAngle;
    radian_t rearWheelTargetAngle;
    radian_t distSensorServoTargetAngle;

    PD_Controller lineController(globals::frontLineCtrl_P_slow, globals::frontLineCtrl_D_slow,
        static_cast<degree_t>(-cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get());

    WatchdogTimer controlDataWatchdog;
    controlDataWatchdog.start(millisecond_t(200));

    while (true) {
        motorPanelLink.update();
        globals::isControlTaskOk = motorPanelLink.isConnected();

        if (motorPanelLink.readAvailable(rxData)) {
            parseMotorPanelData(rxData);
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            controlDataWatchdog.reset();

            if (controlData.directControl) {
                frontWheelTargetAngle = controlData.frontWheelAngle;
                rearWheelTargetAngle = controlData.rearWheelAngle;
            } else {
                // TODO separate line and orientation control for front and rear servo?
                const bool isFwd = globals::car.speed >= m_per_sec_t(0);
                const float speed = max(globals::car.speed, m_per_sec_t(2.0f)).get();
                float P = globals::frontLineCtrl_P_fwd_mul / (speed * speed * speed);
                float D = globals::frontLineCtrl_D_fwd;

                lineController.setParams(P, D);
                lineController.run(static_cast<centimeter_t>(controlData.baseline.pos - controlData.offset).get());

                frontWheelTargetAngle = isFwd ? controlData.angle + degree_t(lineController.getOutput()) : radian_t(0);
                rearWheelTargetAngle = controlData.rearServoEnabled ? controlData.angle - degree_t(lineController.getOutput()) : controlData.angle;
            }

            distSensorServoTargetAngle = frontWheelTargetAngle * globals::distServoTransferRate;

        } else if (controlDataWatchdog.checkTimeout()) {
            controlData.speed = m_per_sec_t(0);
            controlData.rampTime = millisecond_t(0);
        }

        if (motorPanelLink.shouldSend()) {
            fillMotorPanelData(txData, controlData.speed, controlData.rampTime, frontWheelTargetAngle, rearWheelTargetAngle, distSensorServoTargetAngle);
            motorPanelLink.send(txData);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for motor panel UART RxCplt - called when receive finishes.
 */
void micro_MotorPanel_Uart_RxCpltCallback() {
    motorPanelLink.onNewRxData();
}
