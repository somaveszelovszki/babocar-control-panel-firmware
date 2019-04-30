#pragma once

#include <uns/util/types.hpp>

namespace uns {

typedef void adc_handle_t;      // ADC handle - type is bsp library-dependent.
typedef res_id_t adc_channel_t; // ADC channel identifier type.

/* @brief Gets ADC handle by ADC id.
 * @param id The ADC id.
 * @returns Pointer to the correspondent ADC handle.
 **/
adc_handle_t* getADCHandle(res_id_t id);

/* @brief Gets ADC channel by channel id.
 * @param id The channel id.
 * @returns The correspondent ADC channel.
 **/
adc_channel_t getADCChannel(res_id_t id);

/* @brief Sets ADC channel.
 * @param hadc Pointer to the ADC handle.
 * @param channel The channel to set.
 * @returns Status indicating operation success.
 **/
Status ADC_SetChannel(adc_handle_t * const hadc, adc_channel_t channel);

/* @brief Starts ADC and reads value.
 * @param hadc Pointer to the ADC handle.
 * @param pResult Pointer to the result variable.
 * @returns Status indicating operation success.
 **/
Status ADC_ReadValue(adc_handle_t * const hadc, uint32_t *pResult);

} // namespace uns
