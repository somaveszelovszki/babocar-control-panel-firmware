#include <cfg_board.hpp>
#include <micro/container/infinite_buffer.hpp>
#include <micro/container/map.hpp>
#include <micro/control/PID_Controller.hpp>
#include <micro/debug/params.hpp>
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
#include <cfg_track.hpp>

using namespace micro;

extern queue_t<CarProps, 1> carPropsQueue;

CanManager vehicleCanManager(can_Vehicle);

queue_t<ControlData, 1> controlQueue;

namespace {

PID_Params motorControllerParams = { 0.5f, 0.002f, 0.0f };

struct ServoOffsets {
    micro::radian_t front;
    micro::radian_t rear;
    micro::radian_t extra;
};

sorted_map<m_per_sec_t, PID_Params, 20> frontLinePosControllerParams = {
    // speed        P      I      D
    { { 0.00f }, { 0.00f, 0.00f,   0.00f } },
    { { 0.10f }, { 2.40f, 0.00f, 180.00f } },
    { { 1.00f }, { 2.40f, 0.00f, 180.00f } },
    { { 1.50f }, { 1.80f, 0.00f, 180.00f } },
    { { 2.00f }, { 1.70f, 0.00f, 180.00f } },
    { { 2.25f }, { 1.70f, 0.00f, 180.00f } },
    { { 2.50f }, { 1.70f, 0.00f, 180.00f } },
    { { 3.00f }, { 1.30f, 0.00f, 150.00f } },
    { { 3.50f }, { 1.10f, 0.00f, 150.00f } },
    { { 4.00f }, { 0.90f, 0.00f, 150.00f } },
    { { 5.00f }, { 0.65f, 0.00f, 150.00f } },
    { { 6.00f }, { 0.45f, 0.00f, 150.00f } },
    { { 7.00f }, { 0.25f, 0.00f, 150.00f } }
};

sorted_map<m_per_sec_t, PID_Params, 20> rearLineAngleControllerParams = {
    // speed        P      I      D
    { { 0.00f }, { 0.00f, 0.00f, 0.00f  } },
    { { 0.10f }, { 0.80f, 0.00f, 60.00f } },
    { { 1.00f }, { 0.80f, 0.00f, 60.00f } },
    { { 1.50f }, { 0.50f, 0.00f, 60.00f } },
    { { 2.00f }, { 0.40f, 0.00f, 60.00f } },
    { { 2.25f }, { 0.40f, 0.00f, 60.00f } },
    { { 2.50f }, { 0.40f, 0.00f, 60.00f } },
    { { 3.00f }, { 0.20f, 0.00f, 60.00f } },
    { { 3.50f }, { 0.00f, 0.00f, 60.00f } },
    { { 4.00f }, { 0.00f, 0.00f,  0.00f } },
    { { 5.00f }, { 0.00f, 0.00f,  0.00f } },
    { { 6.00f }, { 0.00f, 0.00f,  0.00f } },
    { { 9.00f }, { 0.00f, 0.00f,  0.00f } }
};

constexpr float SERVO_CONTROLLER_MAX_DELTA = static_cast<degree_t>(2 * cfg::WHEEL_MAX_DELTA).get();

PID_Controller frontLinePosController(PID_Params{}, SERVO_CONTROLLER_MAX_DELTA, std::numeric_limits<float>::infinity(), 0.0f);
PID_Controller rearLinePosController(PID_Params{}, SERVO_CONTROLLER_MAX_DELTA, std::numeric_limits<float>::infinity(), 0.0f);

radian_t frontWheelTargetAngle;
radian_t rearWheelTargetAngle;
radian_t frontDistSensorServoTargetAngle;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

ControlData controlData;
MainLine actualLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);
MainLine targetLine(cfg::CAR_FRONT_REAR_SENSOR_ROW_DIST);

PID_Params frontParams = { 1.70f, 0.00f, 100.00f };
PID_Params rearParams  = { 0.40f, 0.00f, 40.00f };

infinite_buffer<std::pair<centimeter_t, degree_t>, 100> prevLineErrors;

void calcTargetAngles(const CarProps& car, const ControlData& controlData) {

    static constexpr uint32_t D_FILTER_SIZE = 20;

    const Sign speedSign = micro::sgn(car.speed);

    targetLine.centerLine.angle = clamp(targetLine.centerLine.angle, -cfg::MAX_TARGET_LINE_ANGLE, cfg::MAX_TARGET_LINE_ANGLE);

    actualLine.centerLine = controlData.lineControl.actual;
    actualLine.updateFrontRearLines();

    targetLine.centerLine = controlData.lineControl.target;
    targetLine.updateFrontRearLines();

    const millimeter_t actualControlLinePos = Sign::POSITIVE == speedSign ? actualLine.frontLine.pos : actualLine.rearLine.pos;
    const millimeter_t targetControlLinePos = Sign::POSITIVE == speedSign ? targetLine.frontLine.pos : targetLine.rearLine.pos;

    const radian_t actualControlAngle = actualLine.centerLine.angle;
    const radian_t targetControlAngle = targetLine.centerLine.angle;

    //frontLinePosController.tune(frontParams);
    frontLinePosController.tune(frontLinePosControllerParams.lerp(abs(car.speed)));

    const std::pair<centimeter_t, degree_t>& peekBackLineError = prevLineErrors.peek_back(D_FILTER_SIZE);

    const centimeter_t posError = targetControlLinePos - actualControlLinePos;
    const degree_t angleError   = targetControlAngle - actualControlAngle;

    const centimeter_t posErrorDiff = (posError - peekBackLineError.first) / D_FILTER_SIZE;
    const degree_t angleErrorDiff   = (angleError - peekBackLineError.second) / D_FILTER_SIZE;

    prevLineErrors.push_back({ posError, angleError });

    frontLinePosController.update(posError.get(), posErrorDiff.get());
    frontWheelTargetAngle = degree_t(frontLinePosController.output()) + targetControlAngle;
    frontWheelTargetAngle = clamp(frontWheelTargetAngle, -cfg::WHEEL_MAX_DELTA, cfg::WHEEL_MAX_DELTA);

    if (controlData.rearSteerEnabled) {
        //rearLinePosController.tune(rearParams);
        rearLinePosController.tune(rearLineAngleControllerParams.lerp(abs(car.speed)));
        rearLinePosController.update(angleError.get(), angleErrorDiff.get());
        rearWheelTargetAngle = degree_t(rearLinePosController.output()) + targetControlAngle;
        rearWheelTargetAngle = clamp(rearWheelTargetAngle, -cfg::WHEEL_MAX_DELTA, cfg::WHEEL_MAX_DELTA);
    } else {
        rearWheelTargetAngle = clamp(targetControlAngle, -cfg::WHEEL_MAX_DELTA, cfg::WHEEL_MAX_DELTA);
    }

    // if the car is going backwards, the front and rear target wheel angles need to be swapped
    if (Sign::NEGATIVE == speedSign) {
        std::swap(frontWheelTargetAngle, rearWheelTargetAngle);
    }

    frontDistSensorServoTargetAngle = cfg::DIST_SENSOR_SERVO_ENABLED ? frontWheelTargetAngle * cfg::DIST_SENSOR_SERVO_TRANSFER_RATE : radian_t(0);
}

void initializeVehicleCan() {
    const CanFrameIds rxFilter = {};
    const CanFrameIds txFilter = {
        can::LongitudinalControl::id(),
        can::LateralControl::id(),
        can::SetMotorControlParams::id()
    };
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runControlTask(void) {

    SystemManager::instance().registerTask();

    initializeVehicleCan();

    WatchdogTimer controlDataWatchdog(millisecond_t(200));

    REGISTER_READ_WRITE_PARAM(motorControllerParams.P);
    REGISTER_READ_WRITE_PARAM(motorControllerParams.I);
    REGISTER_READ_WRITE_PARAM(motorControllerParams.D);

    REGISTER_READ_WRITE_PARAM(frontParams.P);
    REGISTER_READ_WRITE_PARAM(frontParams.I);
    REGISTER_READ_WRITE_PARAM(frontParams.D);

    REGISTER_READ_WRITE_PARAM(rearParams.P);
    REGISTER_READ_WRITE_PARAM(rearParams.I);
    REGISTER_READ_WRITE_PARAM(rearParams.D);

//    char paramName[32];
//    uint32_t i = 0;
//
//    for (std::pair<m_per_sec_t, PID_Params>& param : frontLinePosControllerParams) {
//        sprint(paramName, sizeof(paramName), "front%u_P", i);
//        micro::Params::instance().registerParam(paramName, param.second.P, false, true);
//
//        sprint(paramName, sizeof(paramName), "front%u_I", i);
//        micro::Params::instance().registerParam(paramName, param.second.I, false, true);
//
//        sprint(paramName, sizeof(paramName), "front%u_D", i);
//        micro::Params::instance().registerParam(paramName, param.second.D, false, true);
//
//        ++i;
//    }
//
//    i = 0;
//    for (std::pair<m_per_sec_t, PID_Params>& param : rearLineAngleControllerParams) {
//        sprint(paramName, sizeof(paramName), "rear%u_P", i);
//        micro::Params::instance().registerParam(paramName, param.second.P, false, true);
//
//        sprint(paramName, sizeof(paramName), "rear%u_I", i);
//        micro::Params::instance().registerParam(paramName, param.second.I, false, true);
//
//        sprint(paramName, sizeof(paramName), "rear%u_D", i);
//        micro::Params::instance().registerParam(paramName, param.second.D, false, true);
//
//        ++i;
//    }

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
            controlData.rampTime = millisecond_t(100);
        }

        vehicleCanManager.periodicSend<can::LongitudinalControl>(vehicleCanSubscriberId, controlData.speed, cfg::USE_SAFETY_ENABLE_SIGNAL, controlData.rampTime);
        vehicleCanManager.periodicSend<can::LateralControl>(vehicleCanSubscriberId, frontWheelTargetAngle, rearWheelTargetAngle, frontDistSensorServoTargetAngle);
        vehicleCanManager.periodicSend<can::SetMotorControlParams>(vehicleCanSubscriberId, motorControllerParams.P, motorControllerParams.I);

        SystemManager::instance().notify(!vehicleCanManager.hasTimedOut(vehicleCanSubscriberId) && !controlDataWatchdog.hasTimedOut());
        os_sleep(millisecond_t(1));
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
