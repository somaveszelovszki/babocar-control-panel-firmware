#pragma once

#include <uns/util/types.hpp>

namespace uns {

/* @brief Converts 16-bit signed integer value to bytes.
 * @param value The 16-bit signed integer value to convert to bytes.
 * @param bytes The destination byte array.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
void toBytes(int16_t value, uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

/* @brief Converts 32-bit signed integer value to bytes.
 * @param value The 32-bit signed integer value to convert to bytes.
 * @param bytes The destination byte array.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
void toBytes(int32_t value, uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

/* @brief Converts 32-bit float value to bytes.
 * @param value The 32-bit float value to convert to bytes.
 * @param bytes The destination byte array.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
void toBytes(float32_t value, uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

/* @brief Converts byte array to a 32-bit signed integer value.
 * @param bytes The source byte array.
 * @returns The byte array array as a 32-bit signed integer value.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
int16_t toInt16(const uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

/* @brief Converts byte array to a 32-bit signed integer value.
 * @param bytes The source byte array.
 * @returns The byte array array as a 32-bit signed integer value.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
int32_t toInt32(const uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

/* @brief Converts byte array to a 32-bit float value.
 * @param bytes The source byte array.
 * @returns The byte array array as a 32-bit float value.
 * @param order Defines byte order (LITTLE_ENDIAN_ or BIG_ENDIAN_).
 **/
float32_t toFloat32(const uint8_t bytes[], BitOrder order = BitOrder::LITTLE_ENDIAN_);

uint32_t atoi(const char * const s, int32_t *pResult, uint32_t len = 0);

uint32_t atof(const char * const s, float32_t *pResult, uint32_t len = 0);

/* @brief Converts integer to string.
 * @param n The source integer.
 * @param s The result string.
 * @param numSize The maximum number of characters for the number. @note Does not include the string end character ('\0').
 * @param padding The minimum number of printed digits - 0 by default.
 * @returns Number of characters written.
 **/
uint32_t itoa(int32_t n, char *const s, uint32_t numSize, uint32_t padding = 0);

uint32_t ftoa(float32_t n, char * const s, uint32_t decSize, uint32_t fragSize);

} // namespace uns
