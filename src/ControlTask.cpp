#include <micro/control/PID_Controller.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <cfg_car.hpp>
#include <globals.hpp>

using namespace micro;

CanManager vehicleCanManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(50));

#define CONTROL_QUEUE_LENGTH 1
QueueHandle_t controlQueue = nullptr;
static uint8_t controlQueueStorageBuffer[CONTROL_QUEUE_LENGTH * sizeof(ControlData)];
static StaticQueue_t controlQueueBuffer;

namespace {

PID_Controller linePosController(globals::linePosCtrl_P, 0.0f, globals::linePosCtrl_D, 0.0f,
    static_cast<degree_t>(-cfg::WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);

PID_Controller lineAngleController(globals::lineAngleCtrl_P, 0.0f, globals::lineAngleCtrl_D, 0.0f,
    static_cast<degree_t>(-cfg::WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);

radian_t frontWheelTargetAngle;
radian_t rearWheelTargetAngle;
radian_t frontDistSensorServoTargetAngle;

void calcTargetAngles(const ControlData& controlData) {
    switch (controlData.controlType) {

    case ControlData::controlType_t::Direct:
        frontWheelTargetAngle = controlData.directControl.frontWheelAngle;
        rearWheelTargetAngle = controlData.directControl.rearWheelAngle;
        break;

    case ControlData::controlType_t::Line: {
        // separate line position and angle control for front and rear servos
        const float speed = max(abs(globals::car.speed), m_per_sec_t(1.0f)).get();

        linePosController.tune(globals::linePosCtrl_P / speed, 0.0f, globals::linePosCtrl_D, 0.0f);
        linePosController.update(static_cast<centimeter_t>(controlData.lineControl.actual.pos - controlData.lineControl.desired.pos).get());
        frontWheelTargetAngle = degree_t(linePosController.output()) + controlData.lineControl.desired.angle;

        lineAngleController.tune(globals::lineAngleCtrl_P / speed, 0.0f, globals::lineAngleCtrl_D, 0.0f);
        lineAngleController.update(static_cast<degree_t>(controlData.lineControl.actual.angle - controlData.lineControl.desired.angle).get());
        rearWheelTargetAngle = degree_t(lineAngleController.output()) - controlData.lineControl.desired.angle;

        // if the car is going backwards, the front and rear target wheel angles need to be swapped
        if (globals::car.speed < m_per_sec_t(0)) {
            std::swap(frontWheelTargetAngle, rearWheelTargetAngle);
        }
        break;
    }

    default:
        break;
    }

    frontDistSensorServoTargetAngle = frontWheelTargetAngle * globals::distServoTransferRate;
}

} // namespace

extern "C" void runControlTask(void) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    ControlData controlData;
    canFrame_t rxCanFrame;
    CanFrameHandler vehicleCanFrameHandler;

    vehicleCanFrameHandler.registerHandler(can::LateralState::id(), [] (const uint8_t * const data) {
        radian_t frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(globals::car.frontWheelAngle, globals::car.rearWheelAngle, frontDistSensorServoAngle);
    });

    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(globals::car.speed, globals::car.distance);
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    Timer longitudinalControlTimer(can::LongitudinalControl::period());
    Timer lateralControlTimer(can::LateralControl::period());
    WatchdogTimer controlDataWatchdog(millisecond_t(200));
    Timer sendTimer(millisecond_t(50));

    while (true) {
        globals::isControlTaskOk = !vehicleCanManager.hasRxTimedOut();

        if (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            controlDataWatchdog.reset();
            calcTargetAngles(controlData);

        } else if (controlDataWatchdog.hasTimedOut()) {
            controlData.speed = m_per_sec_t(0);
            controlData.rampTime = millisecond_t(0);
        }

        if (longitudinalControlTimer.checkTimeout()) {
            vehicleCanManager.send(can::LongitudinalControl(controlData.speed, globals::useSafetyEnableSignal, controlData.rampTime));
        }

        if (lateralControlTimer.checkTimeout()) {
            vehicleCanManager.send(can::LateralControl(frontWheelTargetAngle, rearWheelTargetAngle, frontDistSensorServoTargetAngle));
        }

        if (sendTimer.checkTimeout()) {
            LOG_DEBUG("orientation: %f deg", static_cast<degree_t>(globals::car.pose.angle).get());
        }

        os_delay(1);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
