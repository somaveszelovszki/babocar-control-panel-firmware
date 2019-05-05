#include <uns/panel/MotorPanel.hpp>

namespace uns {
namespace panel {

MotorPanel::MotorPanel(uart_handle_t* _huart)
    : huart(_huart) {}

void MotorPanel::enableMotor(bool useSafetyEnableSignal) {
    static uint8_t startChars[2] = { 'R', '\n' };
    static uint8_t startWithSafetyEnableSignalChars[2] = { 'S', '\n' };

    uns::UART_Transmit_DMA(this->huart, useSafetyEnableSignal ? startWithSafetyEnableSignalChars : startChars, 2);
}

}  // namespace panel
}  // namespace uns
