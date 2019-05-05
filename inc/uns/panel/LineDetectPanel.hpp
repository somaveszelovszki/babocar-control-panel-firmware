#pragma once

#include <uns/bsp/uart.hpp>
#include <uns/Line.hpp>

namespace uns {
namespace panel {

class LineDetectPanel {
public:
    LineDetectPanel(uart_handle_t *_huart);

    void startLineSending();

    void getReceivedLinePositions(LinePositions& result) const;

private:
    uart_handle_t *huart;
    uint8_t linesBuffer[1 + cfg::MAX_NUM_LINES];
};

}  // namespace panel
}  // namespace uns
