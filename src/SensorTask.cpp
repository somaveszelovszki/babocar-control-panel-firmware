#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>
#include <micro/panel/MotorPanelData.h>
#include <micro/hw/MPU9250_Gyroscope.hpp>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <cfg_car.hpp>
#include <globals.hpp>

using namespace micro;

namespace {

LineDetectPanel frontLineDetectPanel(uart_FrontLineDetectPanel);
LineDetectPanel rearLineDetectPanel(uart_RearLineDetectPanel);
hw::MPU9250 gyro(i2c_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_250DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);

Timer lineDetectPanelsSendTimer;

void fillLineDetectPanelData(lineDetectPanelDataIn_t& panelData) {
    panelData.flags = globals::indicatorLedsEnabled ? LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED : 0x00;
}

void getLinesFromPanel(LineDetectPanel& panel, LinePositions& positions) {
    lineDetectPanelDataOut_t dataIn = panel.acquireLastValue();
    positions.clear();
    for (uint8_t i = 0; i < dataIn.lines.numLines; ++i) {
        positions.append(millimeter_t(dataIn.lines.values[i].pos_mm));
    }
}

} // namespace

extern "C" void runSensorTask(const void *argument) {
    //LOG_DEBUG("SensorTask running...");

    vTaskDelay(5); // gives time to other tasks to initialize their queues

    Lines lines;
    Line mainLine;

    LowPassFilter<degree_t, 5> gyroAngleFilter;

//    lineDetectPanelDataIn_t frontLineDetectPanelData;
//    fillLineDetectPanelData(frontLineDetectPanelData);
//    frontLineDetectPanel.start(frontLineDetectPanelData);
//
//    lineDetectPanelDataIn_t rearLineDetectPanelData;
//    fillLineDetectPanelData(rearLineDetectPanelData);
//    rearLineDetectPanel.start(rearLineDetectPanelData);

    gyro.initialize();

//    frontLineDetectPanel.waitStart();
//    rearLineDetectPanel.waitStart();
//    lineDetectPanelsSendTimer.start(millisecond_t(100));

    while (true) {

//        if (lineDetectPanelsSendTimer.checkTimeout()) {
//            fillLineDetectPanelData(frontLineDetectPanelData);
//            frontLineDetectPanel.send(frontLineDetectPanelData);
//
//            fillLineDetectPanelData(rearLineDetectPanelData);
//            rearLineDetectPanel.send(rearLineDetectPanelData);
//        }
//
//        LinePositions frontLinePositions;
//        LinePositions rearLinePositions;
//
//        if (frontLineDetectPanel.hasNewValue() && rearLineDetectPanel.hasNewValue()) {
//            getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
//            getLinesFromPanel(rearLineDetectPanel, rearLinePositions);
//        }
//
//        // passes line positions by value to prevent race condition
//        calculateLines(frontLinePositions, rearLinePositions, lines, mainLine);

        const point3<gauss_t> mag = gyro.readMagData();
        if (!isZero(mag.X) || !isZero(mag.Y) || !isZero(mag.Z)) {
            globals::car.pose.angle = normalize360(gyroAngleFilter.update(atan2(mag.Y, mag.X)));
            LOG_DEBUG("orientation: %f deg", static_cast<degree_t>(globals::car.pose.angle).get());
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
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
