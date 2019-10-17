#include <micro/task/common.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/updatable.hpp>
#include <micro/bsp/task.hpp>
#include <micro/bsp/it.hpp>
#include <micro/hw/SteeringServo.hpp>
#include <micro/sensor/Filter.hpp>
#include <micro/control/LineController.hpp>
#include <micro/panel/LineDetectPanel.hpp>
#include <micro/panel/MotorPanel.hpp>
#include <micro/utils/Line.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_os.hpp>
#include <cfg_car.hpp>

#include <globals.hpp>

using namespace micro;

namespace {



} // namespace

extern "C" void runProgRaceTrackTask(const void *argument) {

    while(true) {
        nonBlockingDelay(millisecond_t(1));
    }

    taskDeleteCurrent();
}