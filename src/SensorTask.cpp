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

#include "VL53L1X_api.h"

#include <DetectedLines.hpp>
#include <cfg_car.hpp>
#include <globals.hpp>

using namespace micro;

#define DETECTED_LINES_QUEUE_LENGTH 1
QueueHandle_t detectedLinesQueue;
static uint8_t detectedLinesQueueStorageBuffer[DETECTED_LINES_QUEUE_LENGTH * sizeof(DetectedLines)];
static StaticQueue_t detectedLinesQueueBuffer;

namespace {

LineDetectPanel frontLineDetectPanel(uart_FrontLineDetectPanel);
LineDetectPanel rearLineDetectPanel(uart_RearLineDetectPanel);
LinePatternCalculator linePatternCalc;

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
    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(5); // gives time to other tasks to initialize their queues

    uint16_t dev=0x52;
    int status=0;
    uint8_t byteData, sensorState=0;
    uint16_t wordData;
    uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
    uint16_t Distance;
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum;
    uint8_t RangeStatus;
    uint8_t dataReady;

    status = VL53L1_RdByte(dev, 0x010F, &byteData);
    LOG_DEBUG("VL53L1X Model_ID: %X\n", byteData);
    status = VL53L1_RdByte(dev, 0x0110, &byteData);
    LOG_DEBUG("VL53L1X Module_Type: %X\n", byteData);
    status = VL53L1_RdWord(dev, 0x010F, &wordData);
    LOG_DEBUG("VL53L1X: %X\n", wordData);

    while(sensorState == 0){
        status = VL53L1X_BootState(dev, &sensorState);
        vTaskDelay(2);
    }
    LOG_DEBUG("Chip booted\n");

    /* This function must to be called to initialize the sensor with the default setting  */
    status = VL53L1X_SensorInit(dev);
    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
    status = VL53L1X_SetTimingBudgetInMs(dev, 15); /* in ms possible values [20, 50, 100, 200, 500] */
    status = VL53L1X_SetInterMeasurementInMs(dev, 15); /* in ms, IM must be > = TB */
    //  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
    //  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
    //  status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
    //  status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    LOG_DEBUG("VL53L1X Ultra Lite Driver Example running ...\n");
    status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
    millisecond_t prevDistReadTime;
    while(1){ /* read and display data */
        while (dataReady == 0){
            status = VL53L1X_CheckForDataReady(dev, &dataReady);
            vTaskDelay(2);
        }
        dataReady = 0;
        status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
        status = VL53L1X_GetDistance(dev, &Distance);
        status = VL53L1X_GetSignalRate(dev, &SignalRate);
        status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
        status = VL53L1X_GetSpadNb(dev, &SpadNum);
        status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
        LOG_DEBUG("%fms: %u, %u, %u, %u, %u",
            (getTime() - prevDistReadTime).get(), (uint32_t)RangeStatus, (uint32_t)Distance, (uint32_t)SignalRate, (uint32_t)AmbientRate, (uint32_t)SpadNum);

        prevDistReadTime = getTime();
    }

    while(true) {
        vTaskDelay(50);
    }

//    Lines lines;
//    Line mainLine;
//
//    LowPassFilter<degree_t, 5> gyroAngleFilter;
//
//    lineDetectPanelDataIn_t frontLineDetectPanelData;
//    fillLineDetectPanelData(frontLineDetectPanelData);
//    frontLineDetectPanel.start(frontLineDetectPanelData);
//
//    lineDetectPanelDataIn_t rearLineDetectPanelData;
//    fillLineDetectPanelData(rearLineDetectPanelData);
//    rearLineDetectPanel.start(rearLineDetectPanelData);
//
//    gyro.initialize();
//
//    frontLineDetectPanel.waitStart();
//    rearLineDetectPanel.waitStart();
//    lineDetectPanelsSendTimer.start(millisecond_t(100));
//
//    while (true) {
//
//        const point3<gauss_t> mag = gyro.readMagData();
//        if (!isZero(mag.X) || !isZero(mag.Y) || !isZero(mag.Z)) {
//            globals::car.pose.angle = normalize360(gyroAngleFilter.update(atan2(mag.Y, mag.X)));
//            LOG_DEBUG("orientation: %f deg", static_cast<degree_t>(globals::car.pose.angle).get());
//        }
//
//        if (lineDetectPanelsSendTimer.checkTimeout()) {
//            fillLineDetectPanelData(frontLineDetectPanelData);
//            frontLineDetectPanel.send(frontLineDetectPanelData);
//
//            fillLineDetectPanelData(rearLineDetectPanelData);
//            rearLineDetectPanel.send(rearLineDetectPanelData);
//        }
//
//        if (frontLineDetectPanel.hasNewValue() && rearLineDetectPanel.hasNewValue()) {
//            LinePositions frontLinePositions, rearLinePositions;
//
//            getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
//            getLinesFromPanel(rearLineDetectPanel, rearLinePositions);
//
//            calculateLines(frontLinePositions, rearLinePositions, lines, mainLine);
//            linePatternCalc.update(globals::car.distance, frontLinePositions, rearLinePositions, lines);
//            const DetectedLines detectedLines = { lines, mainLine, linePatternCalc.getPattern() };
//            xQueueOverwrite(detectedLinesQueue, &detectedLines);
//        }
//
//        vTaskDelay(1);
//    }

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
