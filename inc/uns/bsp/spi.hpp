#pragma once

#include <uns/util/units.hpp>

namespace uns {

typedef void spi_handle_t;    // SPI handle - type is bsp library-dependent.

/* @brief Gets SPI handle by SPI id.
 * @param id The SPI id.
 * @returns Pointer to the correspondent SPI handle.
 **/
spi_handle_t* getSPIHandle(res_id_t id);

/* @brief Receives data through SPI. Transfers in blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param rxBuffer The buffer that will store the received data.
 * @param size Number of bytes to exchange.
 * @param timeout The maximum timeout to wait for SPI to be free to use.
 * @return Status indicating operation success.
 **/
Status SPI_Receive(spi_handle_t * const hspi, uint8_t * const rxBuffer, uint32_t size, time_t timeout);

/* @brief Transmits data through SPI. Transfers in blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param txBuffer The buffer storing the data to send.
 * @param size Number of bytes to exchange.
 * @param timeout The maximum timeout to wait for SPI to be free to use.
 * @return Status indicating operation success.
 **/
Status SPI_Transmit(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint32_t size, time_t timeout);

/* @brief Transmits and receives data through SPI. Transfers in blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param txBuffer The buffer storing the data to send.
 * @param rxBuffer The buffer that will store the received data (may be the same as txBuffer).
 * @param size Number of bytes to exchange.
 * @param timeout The maximum timeout to wait for SPI to be free to use.
 * @return Status indicating operation success.
 **/
Status SPI_TransmitReceive(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint8_t * const rxBuffer, uint32_t size, time_t timeout);

/* @brief Receives data through SPI. Transfers in non-blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param rxBuffer The buffer that will store the received data.
 * @param size Number of bytes to exchange.
 * @return Status indicating operation success.
 **/
Status SPI_Receive_IT(spi_handle_t * const hspi, uint8_t * const rxBuffer, uint32_t size);

/* @brief Transmits data through SPI. Transfers in non-blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param txBuffer The buffer storing the data to send.
 * @param size Number of bytes to exchange.
 * @return Status indicating operation success.
 **/
Status SPI_Transmit_IT(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint32_t size);

/* @brief Transmits and receives data through SPI. Transfers in non-blocking mode.
 * @param hspi Pointer to the SPI handle.
 * @param txBuffer The buffer storing the data to send.
 * @param rxBuffer The buffer that will store the received data (may be the same as txBuffer).
 * @param size Number of bytes to exchange.
 * @return Status indicating operation success.
 **/
Status SPI_TransmitReceive_IT(spi_handle_t * const hspi, const uint8_t * const txBuffer, uint8_t * const rxBuffer, uint32_t size);

/* @brief Gets SPI state.
 * @param hspi Pointer to the SPI handle.
 * @return Status indicating SPI state.
 **/
Status SPI_GetState(spi_handle_t * const hspi);

/* @brief Sets SPI state to READY - this way it forces SPI to continue working when stuck in BUSY state.
 * @param hspi Pointer to the SPI handle.
 **/
void SPI_SetReady(spi_handle_t * const hspi);

} // namespace uns
