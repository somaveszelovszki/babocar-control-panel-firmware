#pragma once

#include <uns/bsp/uart.hpp>

namespace uns {
namespace panel {

class MotorPanel {
public:
    MotorPanel(uart_handle_t *_huart);

    Status start(bool useSafetyEnableSignal);

    m_per_sec_t getSpeed() const;

private:
    uart_handle_t *huart;
    uint8_t speedBuffer[4];
};

}  // namespace panel
}  // namespace uns
