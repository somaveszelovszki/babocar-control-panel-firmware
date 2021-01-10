#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>

using namespace micro;

extern queue_t<state_t<char>, 1> radioRecvQueue;

namespace {

void waitStartSignal() {
    state_t<char> startCounter('\0', millisecond_t(0));

    do {
        char prevStartCounter = startCounter.value();
        radioRecvQueue.peek(startCounter, millisecond_t(10));
        if (startCounter.value() != prevStartCounter) {
            LOG_DEBUG("Start counter: %c", startCounter.value());
            prevStartCounter = startCounter.value();
        }
        os_sleep(millisecond_t(50));
    } while (!('0' == startCounter.value() || isBtw(startCounter.value(), 'A', 'Z')));

    LOG_DEBUG("Started!");
}

} // namespace

extern "C" void runStartupTask(void) {
    millisecond_t lastButtonClickTime = getTime();
    uint32_t buttonClick = 0;
    gpioPinState_t prevButtonState = gpioPinState_t::SET;

//    while(0 == buttonClick || getTime() - lastButtonClickTime < second_t(2)) {
//        gpioPinState_t buttonState;
//        gpio_read(gpio_Btn1, buttonState);
//
//        if (gpioPinState_t::RESET == buttonState && gpioPinState_t::SET == prevButtonState) { // detects falling edges
//            ++buttonClick;
//            lastButtonClickTime = getTime();
//            LOG_DEBUG("Click! (%d)", buttonClick);
//        }
//        prevButtonState = buttonState;
//        os_sleep(millisecond_t(50));
//    }

    LOG_DEBUG("Number of clicks: %d", buttonClick);
    SystemManager::instance().setProgramState(/*buttonClick*/enum_cast(cfg::ProgramState::Test));

    if (cfg::ProgramState::WaitStartSignal == static_cast<cfg::ProgramState>(SystemManager::instance().programState())) {
        waitStartSignal();
        SystemManager::instance().setProgramState(enum_cast(cfg::ProgramState::NavigateLabyrinth));
    }
}
