#include <micro/container/map.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/debug/SystemManager.hpp>
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

queue_t<ControlData, 1> controlQueue;

namespace {

PID_Params prevMotorControllerParams, motorControllerParams = { 0.85f, 0.04f, 0.0f };

struct ServoOffsets {
    micro::radian_t frontWheel;
    micro::radian_t rearWheel;
    micro::radian_t extraServo;

    bool operator==(const ServoOffsets& other) const { return frontWheel == other.frontWheel && rearWheel == other.rearWheel && extraServo == other.extraServo; }
    bool operator!=(const ServoOffsets& other) const { return !(*this == other); }
};

ServoOffsets prevServoOffsets, servoOffsets = { degree_t(90), degree_t(90), degree_t(90) };

sorted_map<m_per_sec_t, PID_Params, 10> linePosControllerParams = {
    // speed        P      I      D
    { { 0.0f }, { 0.00f, 0.00f,  0.00f } },
    { { 1.0f }, { 1.50f, 0.00f, 80.00f } },
    { { 1.5f }, { 1.10f, 0.00f, 80.00f } },
    { { 2.0f }, { 0.75f, 0.00f, 80.00f } },
    { { 2.5f }, { 0.60f, 0.00f, 80.00f } },
    { { 3.0f }, { 0.50f, 0.00f, 80.00f } },
    { { 4.0f }, { 0.40f, 0.00f, 80.00f } },
    { { 6.0f }, { 0.25f, 0.00f, 80.00f } },
    { { 9.0f }, { 0.17f, 0.00f, 80.00f } }
};

sorted_map<m_per_sec_t, PID_Params, 10> lineAngleControllerParams = {
    // speed        P      I      D
    { { 0.0f }, { 0.00f, 0.00f,  0.00f } },
    { { 1.0f }, { 1.50f, 0.00f, 80.00f } },
    { { 1.5f }, { 1.10f, 0.00f, 80.00f } },
    { { 2.0f }, { 0.75f, 0.00f, 80.00f } },
    { { 2.5f }, { 0.60f, 0.00f, 80.00f } },
    { { 3.0f }, { 0.50f, 0.00f, 80.00f } },
    { { 4.0f }, { 0.40f, 0.00f, 80.00f } },
    { { 6.0f }, { 0.25f, 0.00f, 80.00f } },
    { { 9.0f }, { 0.17f, 0.00f, 80.00f } }
};

PID_Controller linePosController(PID_Params{}, static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);
PID_Controller lineAngleController(PID_Params{}, static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);

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

        linePosController.tune(linePosControllerParams.lerp(globals::car.speed));
        linePosController.update(static_cast<centimeter_t>(controlData.lineControl.actual.pos - controlData.lineControl.desired.pos).get());
        frontWheelTargetAngle = degree_t(linePosController.output()) + controlData.lineControl.desired.angle;

        lineAngleController.tune(lineAngleControllerParams.lerp(globals::car.speed));
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

    frontDistSensorServoTargetAngle = cfg::DIST_SENSOR_SERVO_ENABLED ? frontWheelTargetAngle * cfg::DIST_SENSOR_SERVO_TRANSFER_RATE : radian_t(0);
}

} // namespace

extern "C" void runControlTask(void) {

    SystemManager::instance().registerTask();

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
        if (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (controlQueue.receive(controlData, millisecond_t(0))) {
            controlDataWatchdog.reset();
            calcTargetAngles(controlData);

        } else if (controlDataWatchdog.hasTimedOut()) {
            controlData.speed = m_per_sec_t(0);
            controlData.rampTime = millisecond_t(0);
        }

        if (longitudinalControlTimer.checkTimeout()) {
            vehicleCanManager.send(can::LongitudinalControl(controlData.speed, cfg::USE_SAFETY_ENABLE_SIGNAL, controlData.rampTime));
        }

        if (lateralControlTimer.checkTimeout()) {
            vehicleCanManager.send(can::LateralControl(frontWheelTargetAngle, rearWheelTargetAngle, frontDistSensorServoTargetAngle));
        }

        if (motorControllerParams != prevMotorControllerParams) {
            vehicleCanManager.send(can::SetMotorControlParams(motorControllerParams.P, motorControllerParams.D));
            prevMotorControllerParams = motorControllerParams;
        }

        if (servoOffsets != prevServoOffsets) {
            vehicleCanManager.send(can::SetFrontWheelParams(servoOffsets.frontWheel, cfg::WHEEL_MAX_DELTA));
            vehicleCanManager.send(can::SetRearWheelParams(servoOffsets.rearWheel, cfg::WHEEL_MAX_DELTA));
            vehicleCanManager.send(can::SetExtraServoParams(servoOffsets.extraServo, cfg::DIST_SENSOR_SERVO_MAX_DELTA));
            prevServoOffsets = servoOffsets;
        }

        if (sendTimer.checkTimeout()) {
            LOG_DEBUG("orientation: %f deg", static_cast<degree_t>(globals::car.pose.angle).get());
        }

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut() && !controlDataWatchdog.hasTimedOut());
        os_delay(1);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
