#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_car.hpp>

using namespace micro;

extern CanManager vehicleCanManager;

extern queue_t<CarProps, 1> carPropsQueue;
queue_t<linePatternDomain_t, 1> linePatternDomainQueue;
queue_t<LineInfo, 1> lineInfoQueue;

namespace {

LineInfo lineInfo;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::FrontLines::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLines*>(data)->acquire(lineInfo.front.lines);
        lineInfoQueue.overwrite(lineInfo);
    });

    vehicleCanFrameHandler.registerHandler(can::RearLines::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLines*>(data)->acquire(lineInfo.rear.lines);
        lineInfoQueue.overwrite(lineInfo);
    });

    vehicleCanFrameHandler.registerHandler(can::FrontLinePattern::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::FrontLinePattern*>(data)->acquire(lineInfo.front.pattern);
        lineInfoQueue.overwrite(lineInfo);
    });

    vehicleCanFrameHandler.registerHandler(can::RearLinePattern::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::RearLinePattern*>(data)->acquire(lineInfo.rear.pattern);
        lineInfoQueue.overwrite(lineInfo);
    });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {
        can::LineDetectControl::id()
    };
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runLineDetectTask(void) {

    SystemManager::instance().registerTask();

    initializeVehicleCan();

    Timer lineDetectControlTimer(can::LineDetectControl::period());

    LineInfo prevLineInfo;

    while (true) {
        CarProps car;
        carPropsQueue.peek(car, millisecond_t(0));

        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        if (lineInfo.front.pattern.type != prevLineInfo.front.pattern.type) {
            LOG_INFO("Front pattern changed from %s to %s", to_string(prevLineInfo.front.pattern.type), to_string(lineInfo.front.pattern.type));
            prevLineInfo.front = lineInfo.front;
        }

        if (lineInfo.rear.pattern.type != prevLineInfo.rear.pattern.type) {
            LOG_INFO("Rear pattern changed from %s to %s", to_string(prevLineInfo.rear.pattern.type), to_string(lineInfo.rear.pattern.type));
            prevLineInfo.rear = lineInfo.rear;
        }

        if (lineDetectControlTimer.checkTimeout()) {
            linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
            linePatternDomainQueue.receive(domain, millisecond_t(0));

            const bool isFwd                     = car.speed >= m_per_sec_t(0);
            const bool isRace                    = linePatternDomain_t::Race == domain;
            const bool isReducedScanRangeEnabled = false;//isRace && ((isFwd && lineInfo.front.lines.size() == 1) || (!isFwd && lineInfo.rear.lines.size() == 1));
            const uint8_t scanRangeRadius        = isReducedScanRangeEnabled ? cfg::REDUCED_LINE_DETECT_SCAN_RADIUS : 0;

            vehicleCanManager.periodicSend<can::LineDetectControl>(vehicleCanSubscriberId, cfg::INDICATOR_LEDS_ENABLED, scanRangeRadius, domain);
        }

        SystemManager::instance().notify(!vehicleCanManager.hasTimedOut(vehicleCanSubscriberId));
        os_sleep(millisecond_t(1));
    }
}
