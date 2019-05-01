#pragma once

#include <uns/util/units.hpp>

namespace uns {

typedef void tim_handle_t;      // Timer handle type - real type is bsp library-dependent.
typedef uint16_t tim_channel_t; // Timer channel identifier type.

/* @brief Gets time since system startup.
 * @returns Time since system startup.
 **/
millisecond_t getTime();

/* @brief Gets exact time since system startup.
 * @returns Exact time since system startup.
 **/
microsecond_t getExactTime();

/* @brief Delays given amount of time. Does not block processor.
 * @param delay The amount of time to delay.
 **/
void nonBlockingDelay(millisecond_t delay);

/* @brief Delays given amount of time. Blocks processor.
 * @param delay The amount of time to delay.
 **/
void blockingDelay(millisecond_t delay);

/* @brief Gets timer handle by timer id.
 * @param id The timer id.
 * @returns Pointer to the correspondent timer handle.
 **/
tim_handle_t* getTimerHandle(res_id_t id);

/* @brief Gets timer channel by channel id.
 * @param id The channel id.
 * @returns The correspondent timer channel.
 **/
tim_channel_t getTimerChannel(res_id_t id);

/* @brief Sets PWM absolute duty cycle.
 * @param htim Pointer to the timer handle.
 * @param channel The timer's channel.
 * @param duty The duty cycle.
 * @return Status indicating operation success.
 **/
Status writePWM(tim_handle_t *htim, tim_channel_t channel, uint32_t duty);

/* @brief Gets counter value.
 * @param htim Pointer to the timer handle.
 * @returns The timer counter's value.
 **/
uint32_t getTimerCounter(tim_handle_t *htim);

/* @brief Sets counter value.
 * @param htim Pointer to the timer handle.
 * @param cntr The timer counter's value.
 **/
void setTimerCounter(tim_handle_t *htim, uint32_t cntr);

/* @brief Gets compare value.
 * @param htim Pointer to the timer handle.
 * @param channel The timer's channel.
 **/
uint32_t getTimerCompare(tim_handle_t *htim, uint32_t channel);

} // namespace uns
