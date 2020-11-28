#include <cfg_board.hpp>
#include <Distances.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/PanelLink.hpp>
#include <micro/panel/DistSensorPanelData.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>


using namespace micro;

queue_t<Distances, 1> distancesQueue;

namespace {

PanelLink<DistSensorPanelOutData, DistSensorPanelInData> frontDistSensorPanelLink(panelLinkRole_t::Master, { uart_FrontDistSensor });
PanelLink<DistSensorPanelOutData, DistSensorPanelInData> rearDistSensorPanelLink(panelLinkRole_t::Master, { uart_RearDistSensor });

Distances distances;

void parseDistSensorPanelData(const DistSensorPanelOutData& rxData, const bool isFront) {

    const meter_t distance = std::numeric_limits<uint16_t>::max() == rxData.distance_mm ? micro::numeric_limits<millimeter_t>::infinity() : millimeter_t(rxData.distance_mm);

    if (isFront) {
        distances.front = distance;
    } else {
        distances.rear = distance;
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

        if (frontDistSensorPanelLink.readAvailable(rxData)) {
            parseDistSensorPanelData(rxData, true);
            distancesQueue.overwrite(distances);
        }

        if (rearDistSensorPanelLink.readAvailable(rxData)) {
            parseDistSensorPanelData(rxData, false);
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
