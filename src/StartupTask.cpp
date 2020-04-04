#include <micro/utils/log.hpp>
#include <micro/utils/time.hpp>

#include <cfg_board.h>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <task.h>

using namespace micro;

namespace {

uint8_t startCounterBuffer[1];
volatile char startCounter = '6';   // start counter will count back from 5 to 0

void waitStartSignal() {
    char prevStartCounter = startCounter;
    HAL_UART_Receive_DMA(uart_RadioModule, startCounterBuffer, 1);

    while ('0' != startCounter) {
        if (startCounter != prevStartCounter) {
            LOG_DEBUG("Seconds until start: %c", startCounter);
            prevStartCounter = startCounter;
        }
        vTaskDelay(50);
    }

    HAL_UART_DMAStop(uart_RadioModule);
    LOG_DEBUG("Started!");
}

} // namespace

extern "C" void runStartupTask(void) {
    millisecond_t lastButtonClickTime = getTime();
    uint32_t buttonClick = 0;
    GPIO_PinState prevButtonState = GPIO_PIN_SET;

    while(0 == buttonClick || getTime() - lastButtonClickTime < second_t(2)) {
        GPIO_PinState buttonState = HAL_GPIO_ReadPin(gpio_Btn, gpioPin_Btn1);

        if (GPIO_PIN_RESET == buttonState && GPIO_PIN_SET == prevButtonState) { // detects falling edges
            ++buttonClick;
            lastButtonClickTime = getTime();
            LOG_DEBUG("Click! (%d)", buttonClick);
        }
        prevButtonState = buttonState;
        vTaskDelay(50);
    }

    LOG_DEBUG("Number of clicks: %d", buttonClick);

    globals::programState = static_cast<ProgramState>(buttonClick);

    if (ProgramState::WaitStartSignal == globals::programState) {
        waitStartSignal();
        globals::programState = ProgramState::NavigateLabyrinth;
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
