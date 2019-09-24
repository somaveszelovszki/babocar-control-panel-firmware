#include <cfg_board.hpp>
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

extern void micro_Command_Uart_RxCpltCallback();                  // Callback for command UART RxCplt - called when receive finishes.
extern void micro_MotorPanel_Uart_RxCpltCallback();               // Callback for motor panel UART RxCplt - called when receive finishes.
extern void micro_RadioModule_Uart_RxCpltCallback();              // Callback for radio module UART RxCplt - called when receive finishes.
extern void micro_FrontLineDetectPanel_Uart_RxCpltCallback();     // Callback for front line detect panel UART RxCplt - called when receive finishes.
extern void micro_RearLineDetectPanel_Uart_RxCpltCallback();      // Callback for rear line detect panel UART RxCplt - called when receive finishes.

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

// TODO already defined in DebugTask
#define MAX_RX_BUFFER_SIZE  512u    // size of the log RX buffer

/* @brief Internal callback - called when UAR receive finishes.
 * @param huart Pointer to the UART handle.
 **/
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == cfg::uart_MotorPanel.ptr) {
        micro_MotorPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_Command.ptr) {
        //uint32_t bytes = MAX_RX_BUFFER_SIZE - ((DMA_HandleTypeDef*)cfg::dma_Bluetooth.handle)->Instance->NDTR;
        micro_Command_Uart_RxCpltCallback();
        ((DMA_TypeDef*)cfg::dma_Command.instance.ptr)->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;    // clears DMA flags before next transfer
        ((DMA_HandleTypeDef*)cfg::dma_Command.handle.ptr)->Instance->NDTR = MAX_RX_BUFFER_SIZE; // sets number of bytes to receive
        ((DMA_HandleTypeDef*)cfg::dma_Command.handle.ptr)->Instance->CR |= DMA_SxCR_EN;         // starts DMA transfer

    } else if (huart == cfg::uart_MotorPanel.ptr) {
        micro_MotorPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_RadioModule.ptr) {
        micro_RadioModule_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_FrontLineDetectPanel.ptr) {
        micro_FrontLineDetectPanel_Uart_RxCpltCallback();

    } else if (huart == cfg::uart_RearLineDetectPanel.ptr) {
        micro_RearLineDetectPanel_Uart_RxCpltCallback();

    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

/* @brief Callback function for timer period elapsed event.
 * @param htim Pointer to the timer handle.
 **/
extern "C" void micro_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // TODO
    micro::Timer::updateAll();
}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}
