#include <micro/control/PD_Controller.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/panel/vehicleCanTypes.hpp>
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

void parseVehicleCanData(const uint32_t id, const uint8_t * const data) {

    switch (id) {
    case can::LateralState::id(): {
        radian_t frontSteeringServoAngle, rearSteeringServoAngle, frontDistSensorServoAngle;
        reinterpret_cast<const can::LateralState*>(data)->acquire(frontSteeringServoAngle, rearSteeringServoAngle, frontDistSensorServoAngle);
        globals::car.frontWheelAngle = frontSteeringServoAngle * cfg::SERVO_WHEEL_TRANSFER_RATE;
        globals::car.rearWheelAngle  = rearSteeringServoAngle * cfg::SERVO_WHEEL_TRANSFER_RATE;
        break;
    }

    case can::LongitudinalState::id():
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(globals::car.speed, globals::car.distance);
        break;
    }
}

} // namespace

extern "C" void runControlTask(void) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    ControlData controlData;

    radian_t frontWheelTargetAngle;
    radian_t rearWheelTargetAngle;
    radian_t frontDistSensorServoTargetAngle;

    PD_Controller lineController(globals::frontLineCtrl_P_slow, globals::frontLineCtrl_D_slow,
        static_cast<degree_t>(-cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get());

    CAN_RxHeaderTypeDef rxHeader;
    alignas(8) uint8_t rxData[8];
    uint32_t txMailbox = 0;

    Timer longitudinalControlTimer(can::LongitudinalControl::period());
    Timer lateralControlTimer(can::LateralControl::period());

    WatchdogTimer vehicleCanWatchdog(millisecond_t(15));
    WatchdogTimer controlDataWatchdog(millisecond_t(200));

    while (true) {
        globals::isControlTaskOk = !vehicleCanWatchdog.hasTimedOut();;

        if (HAL_CAN_GetRxFifoFillLevel(can_Vehicle, canRxFifo_Vehicle)) {
            if (HAL_OK == HAL_CAN_GetRxMessage(can_Vehicle, canRxFifo_Vehicle, &rxHeader, rxData)) {
                parseVehicleCanData(rxHeader.StdId, rxData);
                vehicleCanWatchdog.reset();
            }
        }

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
            CAN_TxHeaderTypeDef txHeader = micro::can::buildHeader<can::LongitudinalControl>();
            can::LongitudinalControl longitudinalControl(controlData.speed, globals::useSafetyEnableSignal, controlData.rampTime);
            HAL_CAN_AddTxMessage(can_Vehicle, &txHeader, reinterpret_cast<uint8_t*>(&longitudinalControl), &txMailbox);
        }

        if (lateralControlTimer.checkTimeout()) {
            CAN_TxHeaderTypeDef txHeader = micro::can::buildHeader<can::LateralControl>();
            can::LateralControl lateralControl(frontWheelTargetAngle, rearWheelTargetAngle, radian_t(0));
            HAL_CAN_AddTxMessage(can_Vehicle, &txHeader, reinterpret_cast<uint8_t*>(&lateralControl), &txMailbox);
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
