#include <uns/panel/handshake.hpp>
#include <uns/util/arrays.hpp>
#include <uns/bsp/tim.hpp>

namespace uns {
namespace panel {

Status handshake(uart_handle_t *huart, const uint8_t *txBuffer, uint32_t txSize) {

    uint8_t rxBuffer[HANDSHAKE_RX_SIZE];
    Status status;

    uns::UART_Stop_DMA(huart);
    if (isOk(status = uns::UART_Transmit(huart, txBuffer, txSize, millisecond_t(2))) &&
        isOk(status = uns::UART_Receive(huart, rxBuffer, HANDSHAKE_RX_SIZE, millisecond_t(2)))) {

        status = uns::isZeroArray(rxBuffer, HANDSHAKE_RX_SIZE) ? Status::OK : Status::INVALID_DATA;
    }

    return status;
}

}  // namespace panel
}  // namespace uns
