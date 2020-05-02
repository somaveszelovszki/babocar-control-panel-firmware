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

extern void micro_Command_Uart_RxCpltCallback(const uint32_t leftBytes);
extern void micro_RadioModule_Uart_RxCpltCallback();
extern void micro_FrontDistSensor_Uart_RxCpltCallback();
extern void micro_RearDistSensor_Uart_RxCpltCallback();
extern void micro_Gyro_CommCpltCallback();

/* @brief Internal callback - called when UAR receive finishes.
 * @param huart Pointer to the UART handle.
 **/
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_Command) {
        micro_Command_Uart_RxCpltCallback(huart->hdmarx->Instance->NDTR);

    } else if (huart == uart_RadioModule) {
        micro_RadioModule_Uart_RxCpltCallback();

    } else if (huart == uart_FrontDistSensor) {
        micro_FrontDistSensor_Uart_RxCpltCallback();

    } else if (huart == uart_RearDistSensor) {
        micro_RearDistSensor_Uart_RxCpltCallback();

    }
}
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == spi_Gyro) {
        micro_Gyro_CommCpltCallback();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == spi_Gyro) {
        micro_Gyro_CommCpltCallback();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == spi_Gyro) {
        micro_Gyro_CommCpltCallback();
    }
}
