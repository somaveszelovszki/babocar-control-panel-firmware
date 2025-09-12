#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"

#include <cfg_board.hpp>
#include <micro/utils/timer.hpp>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void micro_Command_Uart_RxCpltCallback();
extern void micro_Command_Uart_TxCpltCallback();
extern void micro_RadioModule_Uart_RxCpltCallback();
extern void micro_FrontDistSensor_Uart_RxCpltCallback();
extern void micro_RearDistSensor_Uart_RxCpltCallback();
extern void micro_Gyro_CommCpltCallback();
extern void micro_Gyro_DataReadyCallback();
extern void micro_Vehicle_Can_RxFifoMsgPendingCallback();

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == uart_Debug.handle) {
        micro_Command_Uart_RxCpltCallback();
    } else if (huart == uart_RadioModule.handle) {
        micro_RadioModule_Uart_RxCpltCallback();
    } else if (huart == uart_FrontDistSensor.handle) {
        micro_FrontDistSensor_Uart_RxCpltCallback();
    } else if (huart == uart_RearDistSensor.handle) {
        micro_RearDistSensor_Uart_RxCpltCallback();
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == uart_Debug.handle) {
        micro_Command_Uart_TxCpltCallback();
    }
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == spi_Gyro.handle) {
        micro_Gyro_CommCpltCallback();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == spi_Gyro.handle) {
        micro_Gyro_CommCpltCallback();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == spi_Gyro.handle) {
        micro_Gyro_CommCpltCallback();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == can_Vehicle.handle) {
        micro_Vehicle_Can_RxFifoMsgPendingCallback();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        micro_Gyro_DataReadyCallback();
    }
}
