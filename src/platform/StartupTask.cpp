#include <limits>

#include <micro/debug/TaskMonitor.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/ControlData.hpp>
#include <micro/log/log.hpp>
#include <micro/math/numeric.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>
#include <globals.hpp>
#include <ProgramState.hpp>


using namespace micro;

namespace {

void waitStartSignal() {
    uint32_t startCounter = std::numeric_limits<uint32_t>::max();

    ControlData controlData;
    controlData.speed = m_per_sec_t(0);
    controlData.rampTime = millisecond_t(0);
    controlData.rearSteerEnabled = false;
    controlData.lineControl.actual = {};
    controlData.lineControl.target = {};

    while (startCounter != 0) {
        char command[cfg::RADIO_COMMAND_MAX_LENGTH];
        if (radioCommandQueue.receive(command, millisecond_t(0))) {
            const uint32_t c = micro::isBtw(command[0], '0', '5') ? command[0] - '0' : 0;
            if (startCounter != c) {
                startCounter = c;
                LOG_INFO("Start counter: {}", startCounter);
            }
        }

        controlQueue.overwrite(controlData);
        os_sleep(millisecond_t(50));
    };

    LOG_INFO("Started!");
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
    programState.set(static_cast<ProgramState::Value>(buttonClick));

    if (ProgramState::WaitStartSignalRoute == programState.get() || ProgramState::WaitStartSignalRandom == programState.get()) {
        waitStartSignal();
        programState.set(ProgramState::WaitStartSignalRoute == programState.get()
            ? ProgramState::LabyrinthRoute
            : ProgramState::LabyrinthRandom);
    }
}
