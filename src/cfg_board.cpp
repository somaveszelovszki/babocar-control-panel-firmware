#include <cfg_board.hpp>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"

extern "C" TIM_HandleTypeDef  htim1;
extern "C" TIM_HandleTypeDef  htim2;
extern "C" TIM_HandleTypeDef  htim5;
extern "C" UART_HandleTypeDef huart4;
extern "C" UART_HandleTypeDef huart5;
extern "C" UART_HandleTypeDef huart1;
extern "C" UART_HandleTypeDef huart2;
extern "C" UART_HandleTypeDef huart3;
extern "C" UART_HandleTypeDef huart6;
extern "C" DMA_HandleTypeDef hdma_uart4_rx;
extern "C" DMA_HandleTypeDef hdma_uart4_tx;
extern "C" DMA_HandleTypeDef hdma_uart5_rx;
extern "C" DMA_HandleTypeDef hdma_uart5_tx;
extern "C" DMA_HandleTypeDef hdma_usart1_rx;
extern "C" DMA_HandleTypeDef hdma_usart1_tx;
extern "C" DMA_HandleTypeDef hdma_usart2_rx;
extern "C" DMA_HandleTypeDef hdma_usart2_tx;
extern "C" DMA_HandleTypeDef hdma_usart3_rx;
extern "C" DMA_HandleTypeDef hdma_usart3_tx;

namespace cfg {

const micro::tim_handle_t tim_System = { &htim2 };

const micro::tim_handle_t  tim_Servo       = { &htim1 };
const micro::tim_channel_t tim_chnl_Servo1 = { TIM_CHANNEL_1 };
const micro::tim_channel_t tim_chnl_Servo2 = { TIM_CHANNEL_2 };

const micro::uart_handle_t uart_Command              = { &huart1 };
const micro::uart_handle_t uart_MotorPanel           = { &huart2 };
const micro::uart_handle_t uart_RadioModule          = { &huart3 };
const micro::uart_handle_t uart_FrontLineDetectPanel = { &huart4 };
const micro::uart_handle_t uart_RearLineDetectPanel  = { &huart5 };

const micro::dma_struct dma_Command              = { DMA2, &hdma_usart1_rx };
const micro::dma_struct dma_MotorPanel           = { DMA1, &hdma_usart2_rx };
const micro::dma_struct dma_RadioModule          = { DMA1, &hdma_usart3_rx };
const micro::dma_struct dma_FrontLineDetectPanel = { DMA1, &hdma_uart4_rx };
const micro::dma_struct dma_RearLineDetectPanel  = { DMA1, &hdma_uart5_rx };

}
