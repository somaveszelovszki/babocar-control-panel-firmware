#include <micro/utils/Line.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/PD_Controller.hpp>
#include <micro/panel/MotorPanelLink.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.h>
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

MotorPanelLink motorPanelLink(
    uart_MotorPanel,
    millisecond_t(MOTOR_PANEL_LINK_OUT_TIMEOUT_MS),
    millisecond_t(MOTOR_PANEL_LINK_IN_PERIOD_MS));

//hw::SteeringServo frontSteeringServo(
//    tim_SteeringServo, tim_chnl_FrontServo,
//    cfg::FRONT_SERVO_PWM_0, cfg::FRONT_SERVO_PWM_180,
//    cfg::FRONT_SERVO_OFFSET, cfg::FRONT_SERVO_WHEEL_MAX_DELTA,
//    cfg::FRONT_SERVO_WHEEL_TR);
//
//hw::SteeringServo rearSteeringServo(
//    tim_SteeringServo, tim_chnl_RearServo,
//    cfg::REAR_SERVO_PWM_0, cfg::REAR_SERVO_PWM_180,
//    cfg::REAR_SERVO_OFFSET, cfg::REAR_SERVO_WHEEL_MAX_DELTA,
//    cfg::REAR_SERVO_WHEEL_TR);
//
//hw::Servo frontDistServo(
//    tim_ServoX, tim_chnl_ServoX1,
//    cfg::DIST_SERVO_PWM_0, cfg::DIST_SERVO_PWM_180,
//    cfg::DIST_SERVO_OFFSET, cfg::DIST_SERVO_MAX_DELTA);

Timer frontDistServoUpdateTimer;

//meter_t getCarTrajectoryRadius() {
//    const radian_t sumAngle = globals::car.frontWheelAngle - globals::car.rearWheelAngle;
//    return isZero(sumAngle) ? meter_t::infinity() : cfg::CAR_FRONT_REAR_PIVOT_DIST / -tan(sumAngle);
//}

void fillMotorPanelData(motorPanelDataIn_t& txData, m_per_sec_t targetSpeed) {
    txData.controller_P            = globals::motorCtrl_P;
    txData.controller_I            = globals::motorCtrl_I;
    txData.controller_integral_max = globals::motorCtrl_integral_max;
    txData.targetSpeed_mmps        = static_cast<int16_t>(static_cast<mm_per_sec_t>(targetSpeed).get());

    txData.flags = 0x00;
    if (globals::useSafetyEnableSignal) txData.flags |= MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL;
}

static void parseMotorPanelData(motorPanelDataOut_t& rxData) {
    globals::car.distance = millimeter_t(rxData.distance_mm);
    globals::car.speed = mm_per_sec_t(rxData.actualSpeed_mmps);
}

} // namespace

extern "C" void runControlTask(const void *argument) {
    controlQueue = xQueueCreateStatic(CONTROL_QUEUE_LENGTH, sizeof(ControlData), controlQueueStorageBuffer, &controlQueueBuffer);

    vTaskDelay(10); // gives time to other tasks to wake up

    motorPanelDataOut_t rxData;
    motorPanelDataIn_t txData;
    ControlData controlData;
    millisecond_t lastControlDataRecvTime = millisecond_t::zero();

    struct {
        m_per_sec_t prevSpeedRef;
        millisecond_t startTime;
    } ramp;

//    frontSteeringServo.writeWheelAngle(radian_t(0));
//    rearSteeringServo.writeWheelAngle(radian_t(0));
//    frontDistServo.write(radian_t(0));

    PD_Controller lineController(globals::frontLineCtrl_P_slow, globals::frontLineCtrl_D_slow,
        static_cast<degree_t>(-cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get(), static_cast<degree_t>(cfg::FRONT_SERVO_WHEEL_MAX_DELTA).get());

    frontDistServoUpdateTimer.start(millisecond_t(20));

    while (true) {
        motorPanelLink.update();
        globals::isControlTaskOk = motorPanelLink.isConnected();

        if (motorPanelLink.readAvailable(rxData)) {
            parseMotorPanelData(rxData);
        }

        //globals::car.frontWheelAngle = frontSteeringServo.wheelAngle();
        //globals::car.rearWheelAngle = rearSteeringServo.wheelAngle();

        const m_per_sec_t prevSpeedRef = controlData.speed;

        // if no control data is received for a given period, stops motor for safety reasons
        if (xQueueReceive(controlQueue, &controlData, 0)) {
            lastControlDataRecvTime = getTime();

            if (controlData.directControl) {
                //frontSteeringServo.writeWheelAngle(controlData.frontWheelAngle);
                //rearSteeringServo.writeWheelAngle(controlData.rearWheelAngle);
            } else {
                const bool isFwd = globals::car.speed >= m_per_sec_t(0);
                const float speed = max(globals::car.speed, m_per_sec_t(2.0f)).get();
                float P = globals::frontLineCtrl_P_fwd_mul / (speed * speed * speed);
                float D = globals::frontLineCtrl_D_fwd;

                lineController.setParams(P, D);
                lineController.run(static_cast<centimeter_t>(controlData.baseline.pos - controlData.offset).get());

                //frontSteeringServo.writeWheelAngle(isFwd ? controlData.angle + degree_t(lineController.getOutput()) : radian_t(0));
                //rearSteeringServo.writeWheelAngle(controlData.rearServoEnabled ? controlData.angle - degree_t(lineController.getOutput()) : controlData.angle);
            }

        } else if (lastControlDataRecvTime != millisecond_t(0) && getTime() - lastControlDataRecvTime > millisecond_t(1000)) {
            LOG_ERROR("No control data for 1000ms");
            lastControlDataRecvTime = getTime();
            controlData.speed = m_per_sec_t::zero();
            controlData.rampTime = millisecond_t(0);
        }

        if (controlData.speed != prevSpeedRef) {
            ramp.prevSpeedRef = prevSpeedRef;
            ramp.startTime = getTime();
        }

        if (globals::distServoEnabled && frontDistServoUpdateTimer.checkTimeout()) {
            //frontDistServo.write(frontSteeringServo.wheelAngle() * globals::distServoTransferRate);
        }

        if (motorPanelLink.shouldSend()) {
            const m_per_sec_t speedRef = controlData.rampTime > millisecond_t(0) ?
                map(getTime(), ramp.startTime, ramp.startTime + controlData.rampTime, ramp.prevSpeedRef, controlData.speed) :
                controlData.speed;
            fillMotorPanelData(txData, speedRef);
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
