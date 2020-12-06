#pragma once

#include <micro/port/can.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/i2c.hpp>
#include <micro/port/spi.hpp>
#include <micro/port/uart.hpp>

extern CAN_HandleTypeDef  hcan1;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c3;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

#define can_Vehicle             micro::can_t{ &hcan1 }

#define gpio_GyroINT1           micro::gpio_t{ GPIOB, GPIO_PIN_0 }
#define gpio_GyroINT2           micro::gpio_t{ GPIOB, GPIO_PIN_1 }

#define gpio_Led                micro::gpio_t{ GPIOA, GPIO_PIN_5 }

#define gpio_Btn1               micro::gpio_t{ GPIOC, GPIO_PIN_10 }
#define gpio_Btn2               micro::gpio_t{ GPIOC, GPIO_PIN_9 }

#define i2c_X                   micro::i2c_t{ &hi2c3 }

#define spi_Gyro                micro::spi_t{ &hspi1 }
#define csGpio_Gyro             micro::gpio_t{ GPIOB, GPIO_PIN_5 }

#define tim_System              micro::timer_t{ &htim2 }

#define uart_FrontDistSensor    micro::uart_t{ &huart4 }
#define uart_Debug              micro::uart_t{ &huart2 }
#define uart_RadioModule        micro::uart_t{ &huart3 }
#define uart_RearDistSensor     micro::uart_t{ &huart5 }
#define uart_X                  micro::uart_t{ &huart6 }

#define GYRO_MPU9250            1
#define GYRO_LSM6DSO            2
#define GYRO_BOARD              GYRO_MPU9250

#define PANEL_VERSION           0x05

#define QUARTZ_FREQ             megahertz_t(20)
