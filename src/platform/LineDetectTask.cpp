#include <micro/debug/TaskMonitor.hpp>
#include <micro/log/log.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/CarProps.hpp>
#include <micro/utils/LinePattern.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_system.hpp>
#include <globals.hpp>

using namespace micro;

namespace {

LineInfo lineInfo;

CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::Id vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

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
    initializeVehicleCan();
    taskMonitor.registerInitializedTask();

    Timer lineDetectControlTimer(can::LineDetectControl::period());

    LineInfo prevLineInfo;

    while (true) {
        CarProps car;
        carPropsQueue.peek(car, millisecond_t(0));

        while (const auto frame = vehicleCanManager.read(vehicleCanSubscriberId)) {
            vehicleCanFrameHandler.handleFrame(*frame);
        }

        if (lineInfo.front.pattern != prevLineInfo.front.pattern) {
//            LOG_INFO("Front pattern changed to [{} {} {}]",
//                to_string(lineInfo.front.pattern.type), to_string(lineInfo.front.pattern.dir), to_string(lineInfo.front.pattern.side));

            prevLineInfo.front = lineInfo.front;
        }

        if (lineInfo.rear.pattern != prevLineInfo.rear.pattern) {
//            LOG_INFO("Rear pattern changed to [{} {} {}]",
//                to_string(lineInfo.rear.pattern.type), to_string(lineInfo.rear.pattern.dir), to_string(lineInfo.rear.pattern.side));

            prevLineInfo.rear = lineInfo.rear;
        }

        if (lineDetectControlTimer.checkTimeout()) {
            LineDetectControl control;
            lineDetectControlQueue.receive(control, millisecond_t(0));

            vehicleCanManager.periodicSend<can::LineDetectControl>(vehicleCanSubscriberId, cfg::INDICATOR_LEDS_ENABLED,
                control.isReducedScanRangeEnabled ? cfg::REDUCED_LINE_DETECT_SCAN_RADIUS : 0, control.domain);
        }

        const auto ok = !vehicleCanManager.hasTimedOut(vehicleCanSubscriberId);
        taskMonitor.notify(ok);
        os_sleep(millisecond_t(1));
    }
}
