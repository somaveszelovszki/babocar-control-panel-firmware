#include <micro/debug/SystemManager.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <cfg_track.hpp>

using namespace micro;

namespace {

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

void waitStartSignal() {
    char prevStartCounter = startCounter;
    uart_receive(uart_RadioModule, startCounterBuffer, 1);

    while ('0' != startCounter) {
        if (startCounter != prevStartCounter) {
            LOG_DEBUG("Seconds until start: %c", startCounter);
            prevStartCounter = startCounter;
        }
        vTaskDelay(50);
    }

    uart_stopReceive(uart_RadioModule);
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

/* @brief Callback for RadioModule UART RxCplt - called when receive finishes.
 */
void micro_RadioModule_Uart_RxCpltCallback() {
    const uint8_t cntr = static_cast<uint8_t>(startCounterBuffer[0]);
    if (cntr == startCounter - 1) {
        startCounter = cntr;
    }
}
