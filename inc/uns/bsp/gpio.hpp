#pragma once

#include <uns/util/types.hpp>

namespace uns {

typedef void gpio_handle_t;     // GPIO type - bsp library-dependent.
typedef res_id_t gpio_pin_t;    // GPIO pin type - bsp library-dependent.

/* @brief Structure storing GPIO handle and pin.
 **/
struct gpio_pin_struct {
    gpio_handle_t *handle;      // Pointer to the GPIO handle.
    gpio_pin_t pin;             // The GPIO pin.
};

/* @brief GPIO instance ids.
 **/
enum class GPIO : uint8_t {
    A = 'A',     // GPIOA
    B = 'B',     // GPIOB
    C = 'C',     // GPIOC
    D = 'D',     // GPIOD
    E = 'E',     // GPIOE
    F = 'F',     // GPIOF
    G = 'G',     // GPIOG
    H = 'H'      // GPIOH
};

/* @brief Gets GPIO instance by id.
 * @param gpio The GPIO id.
 * @returns Pointer to the correspondent GPIO instance.
 **/
gpio_handle_t* getGPIO(GPIO gpio);

/* @brief Gets GPIO pin by number.
 * @param pin The GPIO pin number.
 * @returns The correspondent GPIO pin.
 **/
gpio_pin_t getGPIOPin(res_id_t pin);

/* @brief Writes GPIO pin.
 * @param gpio Structure containing the GPIO instance and pin.
 * @param pinState Pin state to write.
 **/
void GPIO_WritePin(const gpio_pin_struct& gpio, PinState pinState);

/* @brief Reads GPIO pin.
 * @param gpio Structure containing the GPIO instance and pin.
 * @returns The read pin state.
 **/
PinState GPIO_ReadPin(const gpio_pin_struct& gpio);

/* @brief Toggles GPIO pin.
 * @param gpio Structure containing the GPIO instance and pin.
 **/
void GPIO_TogglePin(const gpio_pin_struct& gpio);
} // namespace uns
