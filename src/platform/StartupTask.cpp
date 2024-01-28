#include "ProgramState.hpp"
#include <micro/debug/TaskMonitor.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/log/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>

using namespace micro;

namespace {

void waitStartSignal() {
    char startCounter = '\0';

    ControlData controlData;
    controlData.speed = m_per_sec_t(0);
    controlData.rampTime = millisecond_t(0);
    controlData.rearSteerEnabled = false;
    controlData.lineControl.actual = {};
    controlData.lineControl.target = {};

    while (startCounter != '0' && !isBtw(startCounter, 'A', 'Z')) {
        char prevStartCounter = startCounter;
        radioRecvQueue.peek(startCounter, millisecond_t(10));

        if (startCounter != prevStartCounter) {
            LOG_DEBUG("Start counter: {}", startCounter);
            prevStartCounter = startCounter;
        }

        controlQueue.overwrite(controlData);
        os_sleep(millisecond_t(50));

    };

    LOG_DEBUG("Started!");
}

} // namespace

extern "C" void runStartupTask(void) {
    millisecond_t lastButtonClickTime = getTime();
    uint32_t buttonClick = 0;
    gpioPinState_t prevButtonState = gpioPinState_t::SET;

    while(0 == buttonClick || getTime() - lastButtonClickTime < second_t(2)) {
        gpioPinState_t buttonState;
        gpio_read(gpio_Btn1, buttonState);

        if (gpioPinState_t::RESET == buttonState && gpioPinState_t::SET == prevButtonState) { // detects falling edges
            ++buttonClick;
            lastButtonClickTime = getTime();
            LOG_DEBUG("Click! ({})", buttonClick);
        }
        prevButtonState = buttonState;
        os_sleep(millisecond_t(50));
    }

    LOG_DEBUG("Number of clicks: {}", buttonClick);
    //programState.set(static_cast<ProgramState::Value>(buttonClick));
    programState.set(ProgramState::ReachSafetyCar);

    if (ProgramState::WaitStartSignal == programState.get()) {
        waitStartSignal();
        programState.set(ProgramState::NavigateLabyrinthRight);
    }
}
