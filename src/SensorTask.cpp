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

void getLinesFromPanel(LineDetectPanel& panel, LinePositions& positions, bool mirror = false) {
    lineDetectPanelDataOut_t dataIn = panel.acquireLastValue();
    positions.clear();
    for (uint8_t i = 0; i < dataIn.lines.numLines; ++i) {
        positions.append(millimeter_t(dataIn.lines.values[i].pos_mm) * (mirror ? -1 : 1));
    }
    positions.removeDuplicates();
}

void testLinePattern() {
    static const m_per_sec_t speed = m_per_sec_t(1);
    static const millisecond_t step = millisecond_t(10);

    typedef vec<LinePositions, 1000> positions_t;

    static const positions_t positions = {
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(-3.8), millimeter_t(0), millimeter_t(3.8) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) },
        { millimeter_t(0) }
    };

    Lines lines;
    Line mainLine;
    LinePositions rearLinePositions;

    meter_t distance = meter_t(0);

    for (uint32_t i = 0; i < positions.size(); ++i) {
        calculateLines(positions[i], lines, mainLine);
        linePatternCalc.update(positions[i], rearLinePositions, lines, distance);
        distance += step * speed;
    }
}

void testLineCalculation() {
    Line mainLine;

    // 0->1 on front panel
    Lines lines = {};
    LinePositions front = { millimeter_t(0) };
    LinePositions rear = {};
    calculateLines(front, rear, lines, mainLine);

    // 0->1 on rear panel
    lines = {};
    front = {};
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 0->1 on both panels
    lines = {};
    front = { millimeter_t(0) };
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 1->0 on front panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) } };
    front = {};
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 1->0 on rear panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) } };
    front = { millimeter_t(0) };
    rear = {};
    calculateLines(front, rear, lines, mainLine);

    // 1->0 on both panels
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) } };
    front = {};
    rear = {};
    calculateLines(front, rear, lines, mainLine);

    // 1->2 on front panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) } };
    front = { millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 1->2 on rear panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) } };
    front = { millimeter_t(0) };
    rear = { millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 2->1 on front panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0) };
    rear = { millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 2->1 on rear panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 2->3 on front panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 2->3 on rear panel
    lines = { { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 3->2 on front panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 3->2 on rear panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 3->1 on front panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0) };
    rear = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 3->1 on rear panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(-40), millimeter_t(0), millimeter_t(40) };
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);

    // 3->1 on front panel, 3->2 on rear panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(0) };
    rear = { millimeter_t(0), millimeter_t(40) };
    calculateLines(front, rear, lines, mainLine);

    // 3->2 on front panel, 3->1 on rear panel
    lines = { { millimeter_t(-40), millimeter_t(-40), radian_t(0) }, { millimeter_t(0), millimeter_t(0), radian_t(0) }, { millimeter_t(40), millimeter_t(40), radian_t(0) } };
    front = { millimeter_t(-40), millimeter_t(0) };
    rear = { millimeter_t(0) };
    calculateLines(front, rear, lines, mainLine);
}

} // namespace

extern "C" void runSensorTask(const void *argument) {

    lineDetectPanelDataIn_t frontLineDetectPanelData, rearLineDetectPanelData;
    Lines lines;
    Line mainLine;
    LowPassFilter<degree_t, 5> gyroAngleFilter;

    detectedLinesQueue = xQueueCreateStatic(DETECTED_LINES_QUEUE_LENGTH, sizeof(DetectedLines), detectedLinesQueueStorageBuffer, &detectedLinesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    frontDistSensor.initialize();

    gyro.initialize();

    frontLineDetectPanel.start();
    rearLineDetectPanel.start();

    frontLineDetectPanel.waitResponse();
    rearLineDetectPanel.waitResponse();

    lineDetectPanelsSendTimer.start(millisecond_t(400));

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
            fillLineDetectPanelData(frontLineDetectPanelData);
            frontLineDetectPanel.send(frontLineDetectPanelData);

            fillLineDetectPanelData(rearLineDetectPanelData);
            rearLineDetectPanel.send(rearLineDetectPanelData);
        }

        if (frontLineDetectPanel.hasNewValue() && rearLineDetectPanel.hasNewValue()) {
            LinePositions frontLinePositions, rearLinePositions;

            getLinesFromPanel(frontLineDetectPanel, frontLinePositions);
            getLinesFromPanel(rearLineDetectPanel, rearLinePositions, true);

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
