#include <uns/panel/LineDetectPanel.hpp>

namespace uns {
namespace panel {

LineDetectPanel::LineDetectPanel(uart_handle_t* _huart)
    : huart(_huart) {}

void LineDetectPanel::startLineSending() {
    static uint8_t startChars[2] = { 'S', '\n' };

    uns::UART_Stop_DMA(this->huart);
    uns::UART_Receive_DMA(this->huart, this->linesBuffer, 1 + cfg::MAX_NUM_LINES);
    uns::UART_Transmit_DMA(this->huart, startChars, 2);
}

void LineDetectPanel::getReceivedLinePositions(LinePositions& result) const {
    result.clear();
    for (uint8_t i = 0; i < this->linesBuffer[0]; ++i) {
        result.append(millimeter_t(this->linesBuffer[1 + i]));
    }
}

}  // namespace panel
}  // namespace uns
