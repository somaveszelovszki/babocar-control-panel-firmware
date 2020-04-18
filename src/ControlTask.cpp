#include <micro/control/PID_Controller.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/panel/CanManager.hpp>
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
#include <queue.h>
#include <task.h>

using namespace micro;

#define CONTROL_QUEUE_LENGTH 1
QueueHandle_t controlQueue;
static uint8_t controlQueueStorageBuffer[CONTROL_QUEUE_LENGTH * sizeof(ControlData)];
static StaticQueue_t controlQueueBuffer;

namespace {

} // namespace

extern "C" void runControlTask(void) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    ControlData controlData;

    radian_t frontWheelTargetAngle;
    radian_t rearWheelTargetAngle;
    radian_t frontDistSensorServoTargetAngle;

    PID_Controller lineController(globals::frontLineCtrl_P_slow, 0.0f, globals::frontLineCtrl_D_slow, 0.0f,
        static_cast<degree_t>(-cfg::FRONT_WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::FRONT_WHEEL_MAX_DELTA).get(), 0.0f);

    Timer longitudinalControlTimer(can::LongitudinalControl::period());
    Timer lateralControlTimer(can::LateralControl::period());

    CanManager canManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(50));

    canManager.registerHandler(can::LateralState::id(), [] (const uint8_t * const data) {
        radian_t frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(globals::car.frontWheelAngle, globals::car.rearWheelAngle, frontDistSensorServoAngle);
    });

    canManager.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(globals::car.speed, globals::car.distance);
    });

    WatchdogTimer controlDataWatchdog(millisecond_t(200));

    while (true) {
        globals::isControlTaskOk = !canManager.hasRxTimedOut();

        canManager.handleIncomingFrames();

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            controlDataWatchdog.reset();

            if (ControlData::controlType_t::Direct == controlData.controlType) {
                frontWheelTargetAngle = controlData.directControl.frontWheelAngle;
                rearWheelTargetAngle = controlData.directControl.rearWheelAngle;

            } else if (ControlData::controlType_t::Line == controlData.controlType) {
                // TODO separate line and orientation control for front and rear servo?
//                const bool isFwd = globals::car.speed >= m_per_sec_t(0);
//                const float speed = max(globals::car.speed, m_per_sec_t(2.0f)).get();
//                float P = globals::frontLineCtrl_P_fwd_mul / (speed * speed * speed);
//                float D = globals::frontLineCtrl_D_fwd;
//
//                lineController.setParams(P, D);
//                lineController.run(static_cast<centimeter_t>(controlData.baseline.pose.pos - controlData.offset).get());
//
//                frontWheelTargetAngle = isFwd ? controlData.baseline.angle + degree_t(lineController.getOutput()) : radian_t(0);
//                rearWheelTargetAngle = controlData.rearServoEnabled ? controlData.angle - degree_t(lineController.getOutput()) : controlData.angle;
            }

            frontDistSensorServoTargetAngle = frontWheelTargetAngle * globals::distServoTransferRate;

        } else if (controlDataWatchdog.hasTimedOut()) {
            controlData.speed = m_per_sec_t(0);
            controlData.rampTime = millisecond_t(0);
        }

        if (longitudinalControlTimer.checkTimeout()) {
            canManager.send(can::LongitudinalControl(controlData.speed, globals::useSafetyEnableSignal, controlData.rampTime));
        }

        if (lateralControlTimer.checkTimeout()) {
            canManager.send(can::LateralControl(frontWheelTargetAngle, rearWheelTargetAngle, frontDistSensorServoTargetAngle));
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
