#include <micro/debug/TaskMonitor.hpp>
#include <micro/panel/PanelLink.hpp>
#include <micro/panel/DistSensorPanelData.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <globals.hpp>

using namespace micro;

#define REAR_DISTANCE_SENSOR_ENABLED true

namespace {

PanelLink<DistSensorPanelOutData, DistSensorPanelInData> frontDistSensorPanelLink(panelLinkRole_t::Master, { uart_FrontDistSensor });

#if REAR_DISTANCE_SENSOR_ENABLED
PanelLink<DistSensorPanelOutData, DistSensorPanelInData> rearDistSensorPanelLink(panelLinkRole_t::Master, { uart_RearDistSensor });
#endif // REAR_DISTANCE_SENSOR_ENABLED

millimeter_t parseDistSensorPanelData(const DistSensorPanelOutData& rxData) {
    return std::numeric_limits<uint16_t>::max() == rxData.distance_mm
            ? micro::numeric_limits<millimeter_t>::infinity()
            : millimeter_t(rxData.distance_mm);
}

bool handleDistanceSensor(
    PanelLink<DistSensorPanelOutData, DistSensorPanelInData>& panelLink,
    micro::queue_t<micro::meter_t, 1>& queue) {
    panelLink.update();

    DistSensorPanelOutData rxData;
    if (panelLink.readAvailable(rxData)) {
        queue.overwrite(parseDistSensorPanelData(rxData));
    }
    
    if (panelLink.shouldSend()) {
        DistSensorPanelInData txData;
        panelLink.send(txData);
    }

    return panelLink.isConnected();
}

} // namespace

extern "C" void runDistSensorTask(void) {
    taskMonitor.registerInitializedTask();

    while (true) {
        const bool frontState = handleDistanceSensor(frontDistSensorPanelLink, frontDistanceQueue);

        #if REAR_DISTANCE_SENSOR_ENABLED
        const bool rearState = handleDistanceSensor(rearDistSensorPanelLink, rearDistanceQueue);
        #else
        const bool rearState = true;
        #endif // REAR_DISTANCE_SENSOR_ENABLED

        taskMonitor.notify(frontState && rearState);
        os_sleep(millisecond_t(1));
    }
}


void micro_FrontDistSensor_Uart_RxCpltCallback() {
    frontDistSensorPanelLink.onNewRxData();
}

void micro_RearDistSensor_Uart_RxCpltCallback() {
#if REAR_DISTANCE_SENSOR_ENABLED
    rearDistSensorPanelLink.onNewRxData();
#endif // REAR_DISTANCE_SENSOR_ENABLED
}
