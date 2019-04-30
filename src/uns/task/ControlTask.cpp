#include <uns/task/common.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/util/debug.hpp>
#include <uns/config/cfg_car.hpp>
#include <uns/config/cfg_track.hpp>
#include <uns/util/units.hpp>
#include <uns/util/unit_utils.hpp>
#include <uns/bsp/queue.hpp>
#include <uns/hw/SteeringServo.hpp>
#include <uns/hw/DC_Motor.hpp>
#include <uns/sensor/CarPropsSensor.hpp>
#include <uns/bsp/task.hpp>
#include <uns/LineController.hpp>
#include <uns/LineData.hpp>
#include <uns/PI_Controller.hpp>
#include <uns/util/convert.hpp>
#include <uns/ControlProps.hpp>

using namespace uns;

extern DebugParams DEBUG_PARAMS;

float32_t debugMotorPWM = 0.0f;

uint32_t rcRecvInSpeed;

CarProps car;
hw::DC_Motor motor(cfg::tim_DC_Motor, cfg::tim_chnl_DC_Fwd, cfg::tim_chnl_DC_Bwd);

namespace {

constexpr float32_t MOTOR_CONTROLLER_Kc = 0.83f;

constexpr time_t period_SpeedController(milliseconds(), 10.0f);
PI_Controller speedController(period_SpeedController, cfg::DC_MOTOR_T_ELECTRICAL, MOTOR_CONTROLLER_Kc, -1.0f, 1.0f);

hw::SteeringServo servo(cfg::tim_Servo, cfg::tim_chnl_Servo1, cfg::SERVO_MID, cfg::WHEEL_MAX_DELTA);

CarPropsSensor carPropsSensor;

ControlProps controlProps;

BounceFilter<uint32_t, 3> rcRecvInSpeedFilter(1.2f, 200);

uint8_t carOriBuffer[4];
volatile float32_t currentCarOri = 0.0f;
volatile angle_t diff = angle_t::ZERO();
volatile bool carOriUpdated = false;

uint8_t carSpeedBuffer[4];

} // namespace

extern "C" void runControlTask(void const *argument) {

    Status status;
    debug::printf(debug::CONTENT_FLAG_LOG, "Initializing CarProps sensor...");
    if (isOk(status = carPropsSensor.initialize())) {
        debug::printf(debug::CONTENT_FLAG_LOG, "CarProps sensor initialized.");
    } else {
        debug::printerr(status, "Error while initializing CarProps sensor!");
        //uns::setErrorFlag();
    }

    uns::GPIO_WritePin(cfg::gpio_GyroReset, PinState::SET);
    uns::nonBlockingDelay(time_t::from<milliseconds>(1));
    uns::GPIO_WritePin(cfg::gpio_GyroReset, PinState::RESET);
    uns::nonBlockingDelay(time_t::from<milliseconds>(100));
    uns::GPIO_WritePin(cfg::gpio_GyroReset, PinState::SET);
    uns::UART_Receive_DMA(cfg::uart_Gyro, carOriBuffer, 4);

    uns::GPIO_WritePin(cfg::gpio_EncoderReset, PinState::SET);
    uns::nonBlockingDelay(time_t::from<milliseconds>(1));
    uns::GPIO_WritePin(cfg::gpio_EncoderReset, PinState::RESET);
    uns::nonBlockingDelay(time_t::from<milliseconds>(100));
    uns::GPIO_WritePin(cfg::gpio_EncoderReset, PinState::SET);
    //uns::I2C_Slave_Receive_DMA(cfg::i2c_Encoder, carSpeedBuffer, 4);
    uns::I2C_Slave_Receive_DMA(cfg::i2c_Encoder, carSpeedBuffer, 4);

    LineController lineController(cfg::WHEEL_BASE, cfg::WHEEL_LED_DIST);

    uns::nonBlockingDelay(time_t::from<milliseconds>(500));
    motor.write(0.0f);
    uns::nonBlockingDelay(time_t::from<milliseconds>(200));

    float32_t prevCarOri = 0.0f;

    time_t prevPropsSendTime = uns::getTime();

    while (!uns::hasErrorHappened()) {
        if (carOriUpdated) {
            carOriUpdated = false;
            carPropsSensor.updateMeas(car); // updates car so that new result will include the corrections made by the other tasks
            const angle_t d_angle = angle_t::from<degrees>(currentCarOri - prevCarOri);
            prevCarOri = currentCarOri;
            if (isOk(status = carPropsSensor.run(d_angle))) {
                car = carPropsSensor.getMeasured();
            } else {
                static uint32_t errCntr = 0;
                //debug::printerr(status, "CarProps sensor read error!");
                if (++errCntr >= 10) {
                    //debug::printerr(status, "Multiple orientation sensor data read failures!");
                    //uns::nonBlockingDelay(time::from<milliseconds>(10));
                    //uns::setErrorFlag();
                }
            }
        }

        if (speedController.shouldRun()) {
            if (DEBUG_PARAMS.speedController.hasValue()) {
                speedController.setParams(DEBUG_PARAMS.speedController.value());
            }

            speedController.setDesired(DEBUG_PARAMS.speed.hasValue() ? DEBUG_PARAMS.speed.value() : controlProps.speed);
            speedController.run(car.speed());
        }

        if (uns::getTime() - prevPropsSendTime >= uns::time_t::from<milliseconds>(500)) {
            //debug::printf(debug::CONTENT_FLAG_CAR_PROPS, "%f|%f|%f|%f", car.pos().X.get<millimeters>(), car.pos().Y.get<millimeters>(), car.speed().get<mm_per_sec>(), car.orientation().get<radians>());
            prevPropsSendTime = uns::getTime();
        }

        if (isOk(status = uns::queueReceive(cfg::queue_ControlProps, &controlProps))) {
            static constexpr float32_t MAGIC = 0.7f;
            car.steeringAngle_ = -lineController.GetControlSignal(car.speed() * MAGIC, controlProps.line.pos, controlProps.line.angle);
            servo.writeWheelAngle(car.steeringAngle());
        }

        //motor.write(0.2f);
        motor.write(speedController.getOutput());
        //motor.write(debugMotorPWM);

        uns::nonBlockingDelay(time_t::from<milliseconds>(2));
    }

    uns::deleteCurrentTask();
}

/* @brief Callback for RcRecv Timer Capture - called when PWM value is captured.
 **/
void uns_RcRecv_Tim_IC_CaptureCallback() {
    rcRecvInSpeed = rcRecvInSpeedFilter.update(uns::getTimerCompare(cfg::tim_RC_Recv, cfg::tim_chnl_RC_Recv2));
}

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void uns_Gyro_Uart_RxCpltCallback() {
    currentCarOri = *reinterpret_cast<float32_t*>(carOriBuffer);
    carOriUpdated = true;
}

/* @brief Called when SPI exchange finishes.
 **/
void uns_Encoder_I2C_RxCpltCallback() {
    static constexpr distance_t INCR_TO_SPEED = (1.0f / INCREMENT_PER_WHEEL_ROT) * cfg::CAR_WHEEL_CIRC;
    static constexpr time_t INCR_DELTA_TIME(milliseconds(), 5);
    int32_t diff = *reinterpret_cast<int32_t*>(carSpeedBuffer);
    car.speed_ = INCR_TO_SPEED * diff / INCR_DELTA_TIME;
    uns::I2C_Slave_Receive_DMA(cfg::i2c_Encoder, carSpeedBuffer, 4);
}
