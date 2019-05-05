#include <uns/globals.hpp>
#include <uns/config/cfg_board.hpp>
#include <uns/config/cfg_os.hpp>
#include <uns/util/debug.hpp>
#include <uns/util/updatable.hpp>
#include <uns/bsp/task.hpp>
#include <uns/hw/SteeringServo.hpp>
#include <uns/sensor/Filter.hpp>
#include <uns/control/LineController.hpp>
#include <uns/panel/LineDetectPanel.hpp>
#include <uns/panel/MotorPanel.hpp>

using namespace uns;

panel::LineDetectPanel frontLineDetectPanel(cfg::uart_FrontLineDetectPanel);
panel::LineDetectPanel rearLineDetectPanel(cfg::uart_RearLineDetectPanel);

namespace {

volatile atomic_updatable<LinePositions> frontLinePositions(cfg::mutex_FrontLinePos);
volatile atomic_updatable<LinePositions> rearLinePositions(cfg::mutex_RearLinePos);

bool isFastSpeedSafe(const Line& line) {
    static constexpr millimeter_t MAX_LINE_POS(85.0f);
    return uns::abs(line.pos_front) <= MAX_LINE_POS && uns::abs(line.pos_rear) <= MAX_LINE_POS;
}

} // namespace

extern "C" void runControlTask(const void *argument) {

    Lines lines;
    Line mainLine;
    LineController lineController(cfg::WHEEL_BASE, cfg::WHEEL_LED_DIST);
    hw::SteeringServo steeringServo(cfg::tim_Servo, cfg::tim_chnl_Servo1, cfg::SERVO_MID, cfg::WHEEL_MAX_DELTA);

    while(!task::hasErrorHappened()) {
        CarProps car;
        globals::car.wait_copy(car);

        if (frontLinePositions.is_updated() && rearLinePositions.is_updated()) {

            LinePositions front, rear;
            frontLinePositions.wait_copy(front);
            rearLinePositions.wait_copy(rear);
            uns::calculateLines(front, rear, lines, mainLine);

            const meter_t baseline = meter_t::ZERO();   // TODO change baseline for more efficient turns
            if (isOk(lineController.run(car.speed, baseline, mainLine))) {
                steeringServo.writeWheelAngle(lineController.getOutput());
            }
        }
    }

    uns::taskDeleteCurrent();
}

/* @brief Callback for motor panel UART RxCplt - called when receive finishes.
 */
void uns_MotorPanel_Uart_RxCpltCallback() {
    // TODO
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void uns_FrontLineDetectPanel_Uart_RxCpltCallback() {
    LinePositions *p = const_cast<LinePositions*>(frontLinePositions.accept_ptr());
    if (p) {
        frontLineDetectPanel.getReceivedLinePositions(*p);
        frontLinePositions.release_ptr();
    }
}

/* @brief Callback for rear line detect panel UART RxCplt - called when receive finishes.
 */
void uns_RearLineDetectPanel_Uart_RxCpltCallback() {
    LinePositions *p = const_cast<LinePositions*>(rearLinePositions.accept_ptr());
    if (p) {
        rearLineDetectPanel.getReceivedLinePositions(*p);
        rearLinePositions.release_ptr();
    }
}



