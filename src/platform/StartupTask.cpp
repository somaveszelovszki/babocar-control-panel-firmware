#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>

using namespace micro;

extern queue_t<char, 1> radioRecvQueue;

namespace {

void waitStartSignal() {
    static char startCounter = '6';   // start counter will count back from 5 to 0
    char prevStartCounter = startCounter;

    do {
        radioRecvQueue.peek(startCounter, millisecond_t(10));
        if (startCounter != prevStartCounter) {
            LOG_DEBUG("Seconds until start: %c", startCounter);
            prevStartCounter = startCounter;
        }
        os_sleep(millisecond_t(50));
    } while ('0' != startCounter);

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
            LOG_DEBUG("Click! (%d)", buttonClick);
        }
        prevButtonState = buttonState;
        os_sleep(millisecond_t(50));
    }

    LOG_DEBUG("Number of clicks: %d", buttonClick);
    SystemManager::instance().setProgramState(buttonClick);

    if (cfg::ProgramState::WaitStartSignal == static_cast<cfg::ProgramState>(SystemManager::instance().programState())) {
        waitStartSignal();
        SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::NavigateLabyrinth));
    }

    vTaskDelete(nullptr);
}
