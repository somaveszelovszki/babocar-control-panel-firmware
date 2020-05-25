#include <cfg_board.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/PanelLink.hpp>
#include <micro/panel/DistSensorPanelData.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <DistancesData.hpp>

using namespace micro;

queue_t<DistancesData, 1> distancesQueue;

namespace {

PanelLink<DistSensorPanelOutData, DistSensorPanelInData> frontDistSensorPanelLink(panelLinkRole_t::Master, { uart_FrontDistSensor });
PanelLink<DistSensorPanelOutData, DistSensorPanelInData> rearDistSensorPanelLink(panelLinkRole_t::Master, { uart_RearDistSensor });

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

    SystemManager::instance().registerTask();

    DistSensorPanelOutData rxData;
    DistSensorPanelInData txData;

    while (true) {
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
            distancesQueue.overwrite(distances);
        }

        if (frontDistSensorPanelLink.shouldSend()) {
            fillDistSensorPanelData(txData);
            frontDistSensorPanelLink.send(txData);
        }

        if (rearDistSensorPanelLink.shouldSend()) {
            fillDistSensorPanelData(txData);
            rearDistSensorPanelLink.send(txData);
        }

        SystemManager::instance().notify(frontDistSensorPanelLink.isConnected() && rearDistSensorPanelLink.isConnected());
        os_sleep(millisecond_t(1));
    }
}


void micro_FrontDistSensor_Uart_RxCpltCallback() {
    frontDistSensorPanelLink.onNewRxData();
}
void micro_RearDistSensor_Uart_RxCpltCallback() {
    rearDistSensorPanelLink.onNewRxData();
}
