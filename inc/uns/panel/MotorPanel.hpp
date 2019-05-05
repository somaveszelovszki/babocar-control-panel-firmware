#pragma once

#include <uns/bsp/uart.hpp>

namespace uns {
namespace panel {

class MotorPanel {
public:
    MotorPanel(uart_handle_t *_huart);

    void enableMotor(bool useSafetyEnableSignal);
private:
    uart_handle_t *huart;
};

}  // namespace panel
}  // namespace uns
