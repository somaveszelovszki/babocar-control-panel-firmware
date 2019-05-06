#include <uns/panel/LineDetectPanel.hpp>
#include <uns/panel/handshake.hpp>

namespace uns {
namespace panel {

LineDetectPanel::LineDetectPanel(uart_handle_t* _huart)
    : huart(_huart) {}

Status LineDetectPanel::start() {
    static constexpr uint8_t startChars[2] = { 'S', '\n' };

    Status status = panel::handshake(this->huart, startChars, 2);
    if (isOk(status)) {
        uns::UART_Receive_DMA(this->huart, this->linesBuffer, 1 + cfg::MAX_NUM_LINES);
    }

    return status;
}

void LineDetectPanel::getLinePositions(LinePositions& result) const {
    result.clear();
    for (uint8_t i = 0; i < this->linesBuffer[0]; ++i) {
        result.append(millimeter_t(this->linesBuffer[1 + i]));
    }
}

}  // namespace panel
}  // namespace uns
