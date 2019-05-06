#include <uns/panel/MotorPanel.hpp>
#include <uns/panel/handshake.hpp>
#include <uns/util/convert.hpp>

namespace uns {
namespace panel {

MotorPanel::MotorPanel(uart_handle_t* _huart)
    : huart(_huart) {}

Status MotorPanel::start(bool useSafetyEnableSignal) {
    static constexpr uint8_t startChars[2] = { 'R', '\n' };
    static constexpr uint8_t startWithSafetyEnableSignalChars[2] = { 'S', '\n' };

    Status status = panel::handshake(this->huart, useSafetyEnableSignal ? startWithSafetyEnableSignalChars : startChars, 2);
    if (isOk(status)) {
        uns::UART_Receive_DMA(this->huart, this->speedBuffer, 4);
    }

    return status;
}

m_per_sec_t MotorPanel::getSpeed() const {
    return m_per_sec_t(uns::toFloat32(this->speedBuffer));
}

}  // namespace panel
}  // namespace uns
