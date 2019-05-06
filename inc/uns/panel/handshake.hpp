#pragma once

#include <uns/bsp/uart.hpp>

namespace uns {
namespace panel {

Status handshake(uart_handle_t *huart, const uint8_t *txBuffer, uint32_t txSize);
}  // namespace panel
}  // namespace uns
