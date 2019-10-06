#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/bsp/task.hpp>
#include <micro/bsp/it.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/LineController.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>

#include <globals.hpp>

using namespace micro;

LineDetectPanel frontLineDetectPanel(uart_FrontLineDetectPanel);
LineDetectPanel rearLineDetectPanel(uart_RearLineDetectPanel);
MotorPanel motorPanel(uart_MotorPanel);

namespace {

Timer motorPanelSendTimer;
Timer lineDetectPanelsSendTimer;

bool isFastSpeedSafe(const Line& line) {
    static constexpr millimeter_t MAX_LINE_POS = centimeter_t(8.5f);
    static constexpr radian_t MAX_LINE_ANGLE = degree_t(8.0f);

    return abs(line.pos_front) <= MAX_LINE_POS && abs(line.angle) <= MAX_LINE_ANGLE;
}

void getLinesFromPanel(const LineDetectPanel& panel, LinePositions& positions) {
    lineDetectPanelDataOut_t dataIn = panel.getLastValue();
    positions.clear();
    for (uint8_t i = 0; i < dataIn.lines.numLines; ++i) {
        positions.append(millimeter_t(dataIn.lines.values[i].pos_mm));
    }
}

} // namespace

extern "C" void runControlTask(const void *argument) {

    while(1) { vTaskDelay(1); }

    Lines lines;
    Line mainLine;

    hw::SteeringServo frontSteeringServo(tim_SteeringServo, tim_chnl_FrontServo, cfg::SERVO_MID_FRONT, cfg::WHEEL_MAX_DELTA_FRONT, cfg::SERVO_WHEEL_TR_FRONT);
    hw::SteeringServo rearSteeringServo(tim_SteeringServo, tim_chnl_RearServo, cfg::SERVO_MID_REAR, cfg::WHEEL_MAX_DELTA_REAR, cfg::SERVO_WHEEL_TR_REAR);

    motorPanelSendTimer.start(millisecond_t(10));
    lineDetectPanelsSendTimer.start(millisecond_t(100));

    while (true) {
        motorPanelDataOut_t motorPanelData = motorPanel.getLastValue();

        enterCritical();
        globals::car.speed = mm_per_sec_t(motorPanelData.actualSpeed_mmps);
        CarProps car = globals::car;
        exitCritical();

        if (motorPanelSendTimer.checkTimeout()) {
            motorPanelDataIn_t data;
            data.targetSpeed_mmps = static_cast<mm_per_sec_t>(globals::targetSpeed).get();
            data.controller_Ti_us = globals::motorController_Ti.get();
            data.controller_Kc    = globals::motorController_Kc;
            data.flags            = globals::useSafetyEnableSignal ? MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL : 0x00;
            motorPanel.send(data);
        }

        if (lineDetectPanelsSendTimer.checkTimeout()) {
            lineDetectPanelDataIn_t dataFront;
            dataFront.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;
            frontLineDetectPanel.send(dataFront);

            lineDetectPanelDataIn_t dataRear;
            dataRear.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;
            rearLineDetectPanel.send(dataRear);
        }

        LinePositions frontLinePositions;
        LinePositions rearLinePositions;

        taskENTER_CRITICAL();
        getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
        getLinesFromPanel(rearLineDetectPanel, rearLinePositions);
        taskEXIT_CRITICAL();

        // passes line positions by value to prevent race condition
        calculateLines(frontLinePositions, rearLinePositions, lines, mainLine);

        if (globals::lineFollowEnabled) {

        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

/* @brief Callback for motor panel UART RxCplt - called when receive finishes.
 */
void micro_MotorPanel_Uart_RxCpltCallback() {
    motorPanel.onDataReceived();
}

/* @brief Callback for front line detect panel UART RxCplt - called when receive finishes.
 */
void micro_FrontLineDetectPanel_Uart_RxCpltCallback() {
    frontLineDetectPanel.onDataReceived();
}

/* @brief Callback for rear line detect panel UART RxCplt - called when receive finishes.
 */
void micro_RearLineDetectPanel_Uart_RxCpltCallback() {
    rearLineDetectPanel.onDataReceived();
}



