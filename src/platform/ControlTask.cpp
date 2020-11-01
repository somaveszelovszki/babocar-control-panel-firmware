#include <cfg_board.hpp>
#include <micro/container/map.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_car.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;

CanManager vehicleCanManager(can_Vehicle, millisecond_t(50));

queue_t<ControlData, 1> controlQueue;

namespace {

PID_Params motorControllerParams = { 0.4f, 0.0f, 0.0f };

struct ServoOffsets {
    micro::radian_t front;
    micro::radian_t rear;
    micro::radian_t extra;
};

ServoOffsets prevServoOffsets, servoOffsets = { degree_t(90), degree_t(90), degree_t(90) };

//6.7cm
//15.1deg

sorted_map<m_per_sec_t, PID_Params, 10> frontLinePosControllerParams = {
    // speed        P      I      D
    { { 0.0f }, { 2.50f, 0.00f, 0.00f } },
    { { 1.0f }, { 2.40f, 0.00f, 0.00f } },
    { { 1.5f }, { 2.25f, 0.00f, 0.00f } },
    { { 2.0f }, { 2.25f, 0.00f, 0.00f } },
    { { 2.5f }, { 2.00f, 0.00f, 0.00f } },
    { { 3.0f }, { 1.50f, 0.00f, 0.00f } },
    { { 4.0f }, { 1.00f, 0.00f, 0.00f } },
    { { 6.0f }, { 0.75f, 0.00f, 0.00f } },
    { { 9.0f }, { 0.45f, 0.00f, 0.00f } }
};

sorted_map<m_per_sec_t, PID_Params, 10> rearLinePosControllerParams = {
    // speed        P      I      D
    { { 0.0f }, { 2.50f, 0.00f, 0.00f } },
    { { 1.0f }, { 2.40f, 0.00f, 0.00f } },
    { { 1.5f }, { 2.25f, 0.00f, 0.00f } },
    { { 2.0f }, { 2.25f, 0.00f, 0.00f } },
    { { 2.5f }, { 2.00f, 0.00f, 0.00f } },
    { { 3.0f }, { 1.50f, 0.00f, 0.00f } },
    { { 4.0f }, { 1.00f, 0.00f, 0.00f } },
    { { 6.0f }, { 0.75f, 0.00f, 0.00f } },
    { { 9.0f }, { 0.45f, 0.00f, 0.00f } }
};

PID_Controller frontLinePosController(PID_Params{}, static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);
PID_Controller rearLinePosController(PID_Params{}, static_cast<degree_t>(cfg::WHEEL_MAX_DELTA).get(), 0.0f);

radian_t frontWheelTargetAngle;
radian_t rearWheelTargetAngle;
radian_t frontDistSensorServoTargetAngle;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

ControlData controlData;
MainLine actualLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
MainLine targetLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

void calcTargetAngles(const CarProps& car, const ControlData& controlData) {
    switch (controlData.controlType) {

    case ControlData::controlType_t::Direct:
        frontWheelTargetAngle = controlData.directControl.frontWheelAngle;
        rearWheelTargetAngle  = controlData.directControl.rearWheelAngle;
        break;

    case ControlData::controlType_t::Line:
    {
        const Sign speedSign = micro::sgn(car.speed);

        actualLine.centerLine = controlData.lineControl.actual;
        actualLine.updateFrontRearLines(speedSign);

        targetLine.centerLine = controlData.lineControl.target;
        targetLine.updateFrontRearLines(speedSign);

        frontLinePosController.tune(frontLinePosControllerParams.lerp(car.speed));
        frontLinePosController.update(static_cast<centimeter_t>(actualLine.frontLine.pos - targetLine.frontLine.pos).get());
        frontWheelTargetAngle = degree_t(frontLinePosController.output()) + targetLine.centerLine.angle;

        rearLinePosController.tune(rearLinePosControllerParams.lerp(car.speed));
        rearLinePosController.update(static_cast<degree_t>(actualLine.centerLine.angle).get());
        rearWheelTargetAngle = degree_t(rearLinePosController.output()) + targetLine.centerLine.angle;

        // if the car is going backwards, the front and rear target wheel angles need to be swapped
        if (Sign::NEGATIVE == speedSign) {
            std::swap(frontWheelTargetAngle, rearWheelTargetAngle);
        }
        break;
    }

    default:
        break;
    }

    frontDistSensorServoTargetAngle = cfg::DIST_SENSOR_SERVO_ENABLED ? frontWheelTargetAngle * cfg::DIST_SENSOR_SERVO_TRANSFER_RATE : radian_t(0);
}

void initializeVehicleCan() {
    const CanFrameIds rxFilter = {};
    const CanFrameIds txFilter = {
        can::LongitudinalControl::id(),
        can::LateralControl::id(),
        can::SetMotorControlParams::id(),
        can::SetFrontSteeringServoParams::id(),
        can::SetRearSteeringServoParams::id(),
        can::SetExtraServoParams::id()
    };
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runControlTask(void) {

    SystemManager::instance().registerTask();

    initializeVehicleCan();

    WatchdogTimer controlDataWatchdog(millisecond_t(200));

    while (true) {
        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        // if no control data is received for a given period, stops motor for safety reasons
        if (controlQueue.receive(controlData, millisecond_t(0))) {
            controlDataWatchdog.reset();
            CarProps car;
            carPropsQueue.peek(car, millisecond_t(0));
            calcTargetAngles(car, controlData);

        } else if (controlDataWatchdog.hasTimedOut()) {
            controlData.speed = m_per_sec_t(0);
            controlData.rampTime = millisecond_t(0);
        }

        vehicleCanManager.periodicSend<can::LongitudinalControl>(vehicleCanSubscriberId, controlData.speed, cfg::USE_SAFETY_ENABLE_SIGNAL, controlData.rampTime);
        vehicleCanManager.periodicSend<can::LateralControl>(vehicleCanSubscriberId, frontWheelTargetAngle, rearWheelTargetAngle, frontDistSensorServoTargetAngle);

        vehicleCanManager.periodicSend<can::SetMotorControlParams>(vehicleCanSubscriberId, motorControllerParams.P, motorControllerParams.D);

        vehicleCanManager.periodicSend<can::SetFrontSteeringServoParams>(vehicleCanSubscriberId, servoOffsets.front, cfg::WHEEL_MAX_DELTA);
        vehicleCanManager.periodicSend<can::SetRearSteeringServoParams>(vehicleCanSubscriberId, servoOffsets.rear, cfg::WHEEL_MAX_DELTA);
        vehicleCanManager.periodicSend<can::SetExtraServoParams>(vehicleCanSubscriberId, servoOffsets.extra, cfg::DIST_SENSOR_SERVO_MAX_DELTA);

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut() && !controlDataWatchdog.hasTimedOut());
        os_sleep(millisecond_t(1));
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
