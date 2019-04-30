#pragma once

#include <uns/util/units.hpp>

namespace uns {

typedef void i2c_handle_t;     // I2C handle - type is bsp library-dependent.

/* @brief Gets I2C handle by I2C id.
 * @param id The I2C id.
 * @returns Pointer to the correspondent I2C handle.
 **/
i2c_handle_t* getI2CHandle(res_id_t id);

/* @brief Checks if I2C device is ready for communication.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param trials Number of trials.
 * @param timeout The timeout duration.
 * @return Status indicating operation success.
 **/
Status I2C_IsDeviceReady(i2c_handle_t * const hi2c, uint16_t address, uint32_t trials, uns::time_t timeout);

/* @brief Transmits data through I2C in master mode. Transmits in blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param txBuffer The buffer storing the data to send.
 * @param size The number of bytes to send.
 * @timeout The maximum timeout to wait for I2C to be free to use.
 **/
Status I2C_Master_Transmit(i2c_handle_t * const hi2c, uint16_t address, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout);

/* @brief Transmits data through I2C in master mode. Transmits in non-blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param txBuffer The buffer storing the data to send.
 * @param size The number of bytes to send.
 **/
Status I2C_Master_Transmit_IT(i2c_handle_t * const hi2c, uint16_t address, const uint8_t * const txBuffer, uint32_t size);

/* @brief Writes to a given memory address through I2C in master mode. Transmits in blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param memAddress The memory address.
 * @param memAddressSize Size of the memory address.
 * @param txBuffer The buffer storing the data to send.
 * @param size The number of bytes to send.
 * @timeout The maximum timeout to wait for I2C to be free to use.
 **/
Status I2C_Mem_Write(i2c_handle_t * const hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, const uint8_t * const txBuffer, uint32_t size, uns::time_t timeout);

/* @brief Receives data through I2C in master mode. Receives in blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param rxBuffer The buffer that will store the received data.
 * @param size The number of bytes to receive.
 * @timeout The maximum timeout to wait for I2C to be free to use.
 **/
Status I2C_Master_Receive(i2c_handle_t * const hi2c, uint16_t address, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout);

/* @brief Receives data through I2C in master mode. Receives in non-blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param rxBuffer The buffer that will store the received data.
 * @param size The number of bytes to receive.
 **/
Status I2C_Master_Receive_IT(i2c_handle_t * const hi2c, uint16_t address, uint8_t * const rxBuffer, uint32_t size);

/* @brief Reads from a given memory address through I2C in master mode. Receives in blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param address The device address.
 * @param memAddress The memory address.
 * @param memAddressSize Size of the memory address.
 * @param rxBuffer The buffer that will store the received data.
 * @param size The number of bytes to receive.
 * @timeout The maximum timeout to wait for I2C to be free to use.
 **/
Status I2C_Mem_Read(i2c_handle_t * const hi2c, uint16_t address, uint16_t memAddress, uint16_t memAddressSize, uint8_t * const rxBuffer, uint32_t size, uns::time_t timeout);

/* @brief Receives data through I2C in slave mode. Receives using DMA.
 * @param hi2c Pointer to the I2C handle.
 * @param rxBuffer The buffer that will store the received data.
 * @param size The number of bytes to receive.
 **/
Status I2C_Slave_Receive_DMA(i2c_handle_t * const hi2c, uint8_t * const rxBuffer, uint32_t size);

/* @brief Receives data through I2C in slave mode. Receives in blocking mode.
 * @param hi2c Pointer to the I2C handle.
 * @param rxBuffer The buffer that will store the received data.
 * @param size The number of bytes to receive.
 * @timeout The maximum timeout to wait for I2C to be free to use.
 **/
Status I2C_Slave_Receive(i2c_handle_t * const hi2c, uint8_t * const rxBuffer, uint32_t size, time_t timeout);
} // namespace uns
