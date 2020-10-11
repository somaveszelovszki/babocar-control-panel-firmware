#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>
#include <DetectedLines.hpp>

using namespace micro;

extern CanManager vehicleCanManager;

extern queue_t<CarProps, 1> carPropsQueue;
queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
queue_t<DetectedLines, 1> detectedLinesQueue;

namespace {

DetectedLines detectedLines;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::FrontLines::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLines*>(data)->acquire(detectedLines.front.lines);
        detectedLinesQueue.overwrite(detectedLines);
    });

    vehicleCanFrameHandler.registerHandler(can::RearLines::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLines*>(data)->acquire(detectedLines.rear.lines);
    });

    vehicleCanFrameHandler.registerHandler(can::FrontLinePattern::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLinePattern*>(data)->acquire(detectedLines.front.pattern);
    });

    vehicleCanFrameHandler.registerHandler(can::RearLinePattern::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLinePattern*>(data)->acquire(detectedLines.rear.pattern);
    });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {};
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runLineDetectTask(void) {

    SystemManager::instance().registerTask();

    initializeVehicleCan();

    Timer lineDetectControlTimer(can::LineDetectControl::period());

    while (true) {
        CarProps car;
        carPropsQueue.peek(car, millisecond_t(0));

        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        if (lineDetectControlTimer.checkTimeout()) {
            linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
            linePatternDomainQueue.receive(domain, millisecond_t(0));

            const bool isFwd                     = car.speed >= m_per_sec_t(0);
            const bool isRace                    = linePatternDomain_t::Race == domain;
            const bool isReducedScanRangeEnabled = isRace && ((isFwd && detectedLines.front.lines.size()) || (!isFwd && detectedLines.rear.lines.size()));
            const uint8_t scanRangeRadius        = isReducedScanRangeEnabled ? cfg::REDUCED_LINE_DETECT_SCAN_RADIUS : 0;

            vehicleCanManager.periodicSend<can::LineDetectControl>(vehicleCanSubscriberId, cfg::INDICATOR_LEDS_ENABLED, scanRangeRadius, domain);
        }

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut());
        os_sleep(millisecond_t(1));
    }
}
