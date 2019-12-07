#include <cfg_board.h>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/LineDetectPanelData.h>
#include <micro/panel/MotorPanel.hpp>
#include <micro/panel/MotorPanelData.h>
#include <micro/hw/MPU9250_Gyroscope.hpp>
#include <micro/hw/VL53L1X_DistanceSensor.hpp>
#include <micro/task/common.hpp>
#include <micro/sensor/Filter.hpp>

#include <ControlData.hpp>
#include <DistancesData.hpp>
#include <DetectedLines.hpp>
#include <cfg_car.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>

using namespace micro;

extern QueueHandle_t distancesQueue;

#define DETECTED_LINES_QUEUE_LENGTH 1
QueueHandle_t detectedLinesQueue;
static uint8_t detectedLinesQueueStorageBuffer[DETECTED_LINES_QUEUE_LENGTH * sizeof(DetectedLines)];
static StaticQueue_t detectedLinesQueueBuffer;

namespace {

LineDetectPanel frontLineDetectPanel(uart_FrontLineDetectPanel);
LineDetectPanel rearLineDetectPanel(uart_RearLineDetectPanel);
LinePatternCalculator linePatternCalc;

hw::MPU9250 gyro(i2c_Gyro, hw::Ascale::AFS_2G, hw::Gscale::GFS_250DPS, hw::Mscale::MFS_16BITS, MMODE_ODR_100Hz);
hw::VL53L1X_DistanceSensor frontDistSensor(i2c_Dist, 0x52);

Timer lineDetectPanelsSendTimer;

void fillLineDetectPanelData(lineDetectPanelDataIn_t& panelData) {
    panelData.flags = 0x00;
    if (globals::indicatorLedsEnabled) panelData.flags |= LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED;
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
    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontDistSensor.initialize();

    Lines lines;
    Line mainLine;

    LowPassFilter<degree_t, 5> gyroAngleFilter;

    frontLineDetectPanel.start();
    rearLineDetectPanel.start();

    gyro.initialize();

    lineDetectPanelsSendTimer.start(millisecond_t(100));

    globals::isSensorTaskInitialized = true;

    while (true) {

        const point3<gauss_t> mag = gyro.readMagData();
        if (!isZero(mag.X) || !isZero(mag.Y) || !isZero(mag.Z)) {
            globals::car.pose.angle = normalize360(gyroAngleFilter.update(atan2(mag.Y, mag.X)));
            //LOG_DEBUG("orientation: %f deg", static_cast<degree_t>(globals::car.pose.angle).get());
        }

        DistancesData distances;
        if (isOk(frontDistSensor.readDistance(distances.front))) {
            xQueueOverwrite(distancesQueue, &distances);
            //LOG_DEBUG("front distance: %u cm", static_cast<uint32_t>(static_cast<centimeter_t>(distances.front).get()));
        }

        if (lineDetectPanelsSendTimer.checkTimeout()) {
            lineDetectPanelDataIn_t frontLineDetectPanelData;
            fillLineDetectPanelData(frontLineDetectPanelData);
            frontLineDetectPanel.send(frontLineDetectPanelData);

            lineDetectPanelDataIn_t rearLineDetectPanelData;
            fillLineDetectPanelData(rearLineDetectPanelData);
            rearLineDetectPanel.send(rearLineDetectPanelData);
        }

        if (frontLineDetectPanel.hasNewValue()/* && rearLineDetectPanel.hasNewValue() */) {
            LinePositions frontLinePositions, rearLinePositions;

            getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
            getLinesFromPanel(rearLineDetectPanel, rearLinePositions);

            calculateLines(frontLinePositions, rearLinePositions, lines, mainLine);
            linePatternCalc.update(frontLinePositions, rearLinePositions, lines, globals::car.distance);
            const DetectedLines detectedLines = { lines, mainLine, linePatternCalc.getPattern() };
            xQueueOverwrite(detectedLinesQueue, &detectedLines);
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
