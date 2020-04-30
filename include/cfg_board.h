#ifndef CFG_BOARD_H
#define CFG_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

extern CAN_HandleTypeDef  hcan1;
extern I2C_HandleTypeDef  hi2c1;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

#define GYRO_MPU9250            1
#define GYRO_LSM6DSO            2
#define GYRO_BOARD              GYRO_LSM6DSO

#if GYRO_BOARD == GYRO_MPU9250
#define i2c_Gyro                (&hi2c1)

#elif GYRO_BOARD == GYRO_LSM6DSO
#define i2c_X                   (&hi2c1)
#define spi_Gyro                (&hspi1)
#define csGpio_Gyro             GPIOB
#define csGpioPin_Gyro          5
#endif

#define can_Vehicle             (&hcan1)
#define canRxFifo_Vehicle       CAN_RX_FIFO0

#define uart_FrontDistSensor    (&huart4)
#define uart_Command            (&huart2)
#define uart_RadioModule        (&huart3)
#define uart_RearDistSensor     (&huart5)
#define uart_X                  (&huart6)

#define gpio_GyroINT            GPIOB
#define gpioPin_GyroINT1        GPIO_PIN_0
#define gpioPin_GyroINT2        GPIO_PIN_1

#define gpio_Led                GPIOA
#define gpioPin_Led             GPIO_PIN_5

#define gpio_Btn                GPIOC
#define gpioPin_Btn1            GPIO_PIN_1
#define gpioPin_Btn2            GPIO_PIN_0

#define tim_System              (&htim2)

#define PANEL_VERSION           0x05

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CFG_BOARD_H
