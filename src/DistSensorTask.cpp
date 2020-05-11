#include <micro/panel/PanelLink.hpp>
#include <micro/panel/DistSensorPanelData.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <DistancesData.hpp>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define DISTANCES_QUEUE_LENGTH 1
QueueHandle_t distancesQueue = nullptr;
static uint8_t distancesQueueStorageBuffer[DISTANCES_QUEUE_LENGTH * sizeof(DistancesData)];
static StaticQueue_t distancesQueueBuffer;

namespace {

PanelLink<DistSensorPanelOutData, DistSensorPanelInData> frontDistSensorPanelLink(panelLinkRole_t::Master, uart_FrontDistSensor);
PanelLink<DistSensorPanelOutData, DistSensorPanelInData> rearDistSensorPanelLink(panelLinkRole_t::Master, uart_RearDistSensor);

DistancesData distances;

void parseDistSensorPanelData(const DistSensorPanelOutData& rxData, const bool isFront) {
    if (isFront) {
        distances.front = millimeter_t(rxData.distance_mm);
    } else {
        distances.rear = millimeter_t(rxData.distance_mm);
    }
}

void fillDistSensorPanelData(DistSensorPanelInData& txData) {
    UNUSED(txData);
}

} // namespace

extern "C" void runDistSensorTask(void) {

    distancesQueue = xQueueCreateStatic(DISTANCES_QUEUE_LENGTH, sizeof(DistancesData), distancesQueueStorageBuffer, &distancesQueueBuffer);

    vTaskDelay(300); // gives time to other tasks to wake up

    DistSensorPanelOutData rxData;
    DistSensorPanelInData txData;

    while (true) {
        globals::isDistSensorTaskOk = frontDistSensorPanelLink.isConnected() && rearDistSensorPanelLink.isConnected();

        frontDistSensorPanelLink.update();
        rearDistSensorPanelLink.update();

        bool updated = false;

        if (frontDistSensorPanelLink.readAvailable(rxData)) {
            parseDistSensorPanelData(rxData, true);
            updated = true;
        }

        if (rearDistSensorPanelLink.readAvailable(rxData)) {
            parseDistSensorPanelData(rxData, false);
            updated = true;
        }

        if (updated) {
            xQueueOverwrite(distancesQueue, &distances);
        }

        if (frontDistSensorPanelLink.shouldSend()) {
            fillDistSensorPanelData(txData);
            frontDistSensorPanelLink.send(txData);
        }

        if (rearDistSensorPanelLink.shouldSend()) {
            fillDistSensorPanelData(txData);
            rearDistSensorPanelLink.send(txData);
        }

        os_delay(1);
    }
}


void micro_FrontDistSensor_Uart_RxCpltCallback() {
    frontDistSensorPanelLink.onNewRxData();
}
void micro_RearDistSensor_Uart_RxCpltCallback() {
    rearDistSensorPanelLink.onNewRxData();
}
