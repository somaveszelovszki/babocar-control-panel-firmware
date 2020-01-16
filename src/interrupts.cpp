#include <cfg_board.h>
#include <micro/utils/timer.hpp>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void micro_Command_Uart_RxCpltCallback(const uint32_t leftBytes);    // Callback for command UART RxCplt - called when receive finishes.
extern void micro_Command_Uart_TxCpltCallback();                            // Callback for command UART TxCplt - called when transmit finishes.
extern void micro_MotorPanel_Uart_RxCpltCallback();                         // Callback for motor panel UART RxCplt - called when receive finishes.
extern void micro_RadioModule_Uart_RxCpltCallback();                        // Callback for radio module UART RxCplt - called when receive finishes.
extern void micro_FrontLineDetectPanel_Uart_RxCpltCallback();               // Callback for front line detect panel UART RxCplt - called when receive finishes.

/* @brief Internal callback - called when SPI reception finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

}

/* @brief Internal callback - called when SPI transmission finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

}

/* @brief Internal callback - called when SPI exchange finishes.
 * @param hspi Pointer to the SPI handle.
 **/
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

}

/* @brief Internal callback - called when I2C reception finishes.
 * @param hi2c Pointer to the I2C handle.
 **/
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

/* @brief Internal callback - called when UAR receive finishes.
 * @param huart Pointer to the UART handle.
 **/
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_MotorPanel) {
        micro_MotorPanel_Uart_RxCpltCallback();

    } else if (huart == uart_Command) {
        micro_Command_Uart_RxCpltCallback(huart->hdmarx->Instance->NDTR);

    } else if (huart == uart_RadioModule) {
        micro_RadioModule_Uart_RxCpltCallback();

    } else if (huart == uart_FrontLineDetectPanel) {
        micro_FrontLineDetectPanel_Uart_RxCpltCallback();

    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_Command) {
        micro_Command_Uart_TxCpltCallback();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {}

/* @brief Callback function for timer period elapsed event.
 * @param htim Pointer to the timer handle.
 **/
extern "C" void micro_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    micro::Timer::updateAll();
}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}
