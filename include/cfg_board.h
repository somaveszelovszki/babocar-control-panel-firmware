#ifndef CFG_BOARD_H
#define CFG_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"

extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart2_tx;
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern DMA_HandleTypeDef  hdma_usart3_tx;
extern DMA_HandleTypeDef  hdma_uart4_rx;
extern DMA_HandleTypeDef  hdma_uart4_tx;
extern DMA_HandleTypeDef  hdma_uart5_rx;
extern DMA_HandleTypeDef  hdma_uart5_tx;
extern DMA_HandleTypeDef  hdma_usart6_rx;
extern DMA_HandleTypeDef  hdma_usart6_tx;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;

#define tim_System (&htim2)

#define tim_SteeringServo            (&htim1)
#define tim_chnl_FrontServo          TIM_CHANNEL_1
#define tim_chnl_RearServo           TIM_CHANNEL_2

#define tim_ServoX                   (&htim3)
#define tim_chnl_ServoX1             TIM_CHANNEL_4
#define tim_chnl_ServoX2             TIM_CHANNEL_3

#define uart_MotorPanel              (&huart1)
#define uart_Command                 (&huart2)
#define uart_RadioModule             (&huart3)
#define uart_X1                      (&huart4)
#define uart_RearLineDetectPanel     (&huart5)
#define uart_FrontLineDetectPanel    (&huart6)

#define i2c_Gyro                     (&hi2c1)
#define i2c_Dist                     (&hi2c2)

#define gpio_Led                     GPIOA
#define gpioPin_Led                  GPIO_PIN_5

#define gpio_Btn                     GPIOC
#define gpioPin_Btn1                 GPIO_PIN_1
#define gpioPin_Btn2                 GPIO_PIN_0

#define gpio_GyroEn                  GPIOB
#define gpioPin_GyroEn               GPIO_PIN_0

#define gpio_DistEn                  GPIOB
#define gpioPin_DistEn               GPIO_PIN_1

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CFG_BOARD_H
